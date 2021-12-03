// /*!
//  * @file     dynamics_raisim.cpp
//  * @author   Giuseppe Rizzi
//  * @date     05.10.2020
//  * @version  1.0
//  * @brief    description
//  */

#include "mppi_sliding/dynamics.h"
#include <ros/package.h>

namespace manipulation {

PandaRaisimDynamics::PandaRaisimDynamics(const DynamicsParams& params, const bool if_sim)   
    : params_(params),if_sim_(if_sim) {

  if(if_sim_)
  {
    std::cout << "this dynamics is as [REAL WORLD] dynamics" << std::endl;
  }
  if(!if_sim_)
  {
    std::cout << "this dynamics is as [MODEL ESTIMATED] dynamics" << std::endl;
    if_update_ = true;
    if(if_update_)
    {
      std::cout << "activate model updater " << std::endl;
    }
  }
  
  initialize_world(params_.robot_description, 
                  params_.object_description, 
                  params_.cylinder_description,
                  params_.mug_description);
  initialize_pd();
  set_collision();

  t_ = 0.0;
  ee_force_applied_ = false;

};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    const std::string& object_description,
    const std::string& cylinder_description,
    const std::string& mug_description) {

  // Raisim world 
  dt_ = params_.dt;
  sim_.setTimeStep(params_.dt);
  sim_.setERP(0., 0.);
  gravity_.e() << 0.0, 0.0, -9.81;
  sim_.setGravity(gravity_);
  sim_.setMaterialPairProp("steel", "steel", params_.friction, 0.15, 0.001); //set friction properties
  
  // create robot
  robot_description_ = robot_description;
  panda_ = sim_.addArticulatedSystem(robot_description_, "/");
  tau_ext_ = Eigen::VectorXd::Zero(panda_->getDOF());
  J_contact_.setZero(3, panda_->getDOF());
  panda_->setName("Panda");

  /// create raisim objects
  // object_description_ = object_description;
  // object_ = sim_.addArticulatedSystem(object_description_, "/");
  // object_->setName("Door");
  // std::cout << "robot and obj inited" << std::endl;

  //  create cylinder object
  if(!if_sim_)
  {
    cylinder_description_ = cylinder_description;
    cylinder_ = sim_.addCylinder(params_.cylinder_radius, params_.cylinder_height,
                0.5,"steel", raisim::COLLISION(1), -1); 
    cylinder_->setMass(params_.cylinder_mass);
    cylinder_->setBodyType(raisim::BodyType::DYNAMIC);
    cylinder_->setName("Cylinder");
  }

  // create mug 
  if(if_sim_)
  {
    mug_description_ = mug_description;
    mug_ = sim_.addArticulatedSystem(mug_description_, "/",{},
                      raisim::COLLISION(1), -1);
  }

  
  // To simulate 2D sliding, we need a table 
  table_ = sim_.addBox(3,3,0.2,10,"steel",
          raisim::COLLISION(1), -1);
  table_->setBodyType(raisim::BodyType::STATIC); //no velocity, inf mass
  table_->setName("Table");
  
  // state size init, according to DOF
  robot_dof_ = BASE_ARM_GRIPPER_DIM;
  state_dimension_ = STATE_DIMENSION;
  input_dimension_ = INPUT_DIMENSION;
  x_ = mppi::observation_t::Zero(state_dimension_);
  


  // init state setup
  reset(params_.initial_state, t_);

  std::cout << "table inited at: " << table_->getPosition() << std::endl; 
  if (if_sim_)
    std::cout << "mug inited at: " << mug_->getGeneralizedCoordinate() << std::endl; 
  if (!if_sim_)
    std::cout << "cylinder inited at: " << cylinder_->getPosition() << std::endl; 
  
}

void PandaRaisimDynamics::initialize_pd() {
  /// panda
  cmd_.setZero(robot_dof_);
  cmdv_.setZero(robot_dof_);
  joint_p_.setZero(robot_dof_);
  joint_v_.setZero(robot_dof_);
  joint_p_gain_.setZero(robot_dof_);
  joint_d_gain_.setZero(robot_dof_);
  joint_p_desired_.setZero(robot_dof_);
  joint_v_desired_.setZero(robot_dof_);

  // clang-format off
  joint_p_gain_.head(BASE_DIMENSION) = params_.gains.base_gains.Kp;
  joint_d_gain_.head(BASE_DIMENSION) = params_.gains.base_gains.Kd;
  joint_p_gain_.segment(BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kp;
  joint_d_gain_.segment(BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kd;
  joint_p_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kp;
  joint_d_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kd;

  // clang-format on

  panda_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);\
  std::cout << "panda dof: " << panda_->getDOF() << std::endl;
  panda_->setPdGains(joint_p_gain_, joint_d_gain_);
  panda_->setGeneralizedForce(Eigen::VectorXd::Zero(panda_->getDOF()));

  // why set PD for object?
  // object_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  // object_->setPdGains(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
  // object_->setGeneralizedForce({0.0});

}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : panda_->getBodyNames())
    pandaBodyIdxs.push_back(panda_->getBodyIdx(bodyName));
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      panda_->ignoreCollisionBetween(body_idx1, body_idx2);
}

void PandaRaisimDynamics::set_control(const mppi::input_t& u) {
  // keep the gripper in the current position

  cmd_.tail<PandaDim::GRIPPER_DIMENSION>()
      << x_.head<BASE_ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();

  // base
  // cmdv_(0) = u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
  // cmdv_(1) = u(0) * std::sin(x_(2)) + u(1) * std::cos(x_(2));
  // cmdv_(2) = u(2);

  // arm
  cmdv_.segment<ARM_DIMENSION>(BASE_DIMENSION) = u.segment<ARM_DIMENSION>(BASE_DIMENSION);

  if(if_sim_)
  {
      //ROS_INFO_STREAM("real input: " << u.transpose());  
      cmdv_(0) = ((double) rand() / (RAND_MAX)) + 1;
      // cmdv_(2) = ((double) rand() / (RAND_MAX)) - 0.5;
      // cmdv_(3) = ((double) rand() / (RAND_MAX)) - 0.5 ;
      // cmdv_(4) = ((double) rand() / (RAND_MAX)) - 0.5 ;
      // cmdv_(5) = ((double) rand() / (RAND_MAX)) - 0.5 ;
      // cmdv_(6) = ((double) rand() / (RAND_MAX)) -0.5 ;
      // cmdv_(7) = ((double) rand() / (RAND_MAX)) -0.5 ;
      // cmdv_(0) = ((double) rand() / (RAND_MAX)) -0.5 ;

      cmdv_(0) = 2*(cmdv_(0));
  }


  if(!if_sim_)
  {
      //ROS_INFO_STREAM("primitive input: " << u.transpose());  
  }


  // gripper
  cmdv_.tail<PandaDim::GRIPPER_DIMENSION>().setZero();
  
  // PD low-level controller
  panda_->setPdTarget(cmd_, cmdv_);

  panda_->setGeneralizedForce(panda_->getNonlinearities(gravity_));

  // gravity compensated object
  //object_->setGeneralizedForce(object_->getNonlinearities(gravity_));
  
}

void PandaRaisimDynamics::advance() {
  // get contact state
  double in_contact = -1;

  if(if_sim_)
  {
    for (const auto& contact : mug_->getContacts()) {
      if (!contact.skip() && !contact.isSelfCollision()) {
        in_contact = 1;
        break;
      }
    }
    // get external torque
    get_external_torque(tau_ext_);

    // step simulation
    sim_.integrate();
    t_ += sim_.getTimeStep();

    // update state x
    panda_->getState(joint_p_, joint_v_);

    //object_->getState(object_p_, object_v_);
    object_p_.resize(OBJECT_DIMENSION);
    object_p_(0) = mug_->getGeneralizedCoordinate().e()[0];
    object_p_(1) = mug_->getGeneralizedCoordinate().e()[1];
    object_p_(2) = mug_->getGeneralizedCoordinate().e()[2];  // for mug this is z axis 
    object_p_(3) = mug_->getGeneralizedCoordinate().e()[3];  
    object_p_(4) = mug_->getGeneralizedCoordinate().e()[4];  
    object_p_(5) = mug_->getGeneralizedCoordinate().e()[5];  
    object_p_(6) = mug_->getGeneralizedCoordinate().e()[6]; 
    
    object_v_.resize(OBJECT_DIMENSION);
    object_v_.setZero();   //TODO: update real rot vel here
    object_v_(0) = mug_->getGeneralizedVelocity().e()[0];
    object_v_(1) = mug_->getGeneralizedVelocity().e()[1];
    object_v_(2) = mug_->getGeneralizedVelocity().e()[2];
  }

  if(!if_sim_)
  {
    for (const auto& contact : cylinder_->getContacts()) {
      if (!contact.skip() && !contact.isSelfCollision()) {
        //std::cout << "this contact position: " <<contact.getPosition().e().transpose()<<std::endl;
        in_contact = 1;
        break;
      }
    }

    // get external torque
    get_external_torque(tau_ext_);

    // step simulation
    sim_.integrate();
    t_ += sim_.getTimeStep();

    // update state x
    panda_->getState(joint_p_, joint_v_);
    //object_->getState(object_p_, object_v_);
    object_p_.resize(OBJECT_DIMENSION);
    object_p_(0) = cylinder_->getPosition()(0);
    object_p_(1) = cylinder_->getPosition()(1);
    object_p_(2) = cylinder_->getPosition()(2);
    object_p_(3) = cylinder_->getQuaternion()(0);
    object_p_(4) = cylinder_->getQuaternion()(1);  
    object_p_(5) = cylinder_->getQuaternion()(2);  
    object_p_(6) = cylinder_->getQuaternion()(3);
    
    object_v_.resize(OBJECT_DIMENSION);
    object_v_.setZero();   //TODO: update real rot vel here
    object_v_(0) = cylinder_->getLinearVelocity()(0);
    object_v_(1) = cylinder_->getLinearVelocity()(1);
    object_v_(2) = cylinder_->getLinearVelocity()(2);

    // update model depending on the flag
    if(if_update_)
    {
      update_model();
    }

  }

  x_.head<BASE_ARM_GRIPPER_DIM>() = joint_p_;
  //x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM) = joint_v_;
  x_.segment(robot_dof_, robot_dof_) = joint_v_;
  //x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM) = object_p_;
  x_.segment(2 * robot_dof_, OBJECT_DIMENSION) = object_p_;
  //x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION) = object_v_;
  x_.segment(2 * robot_dof_ + OBJECT_DIMENSION, OBJECT_DIMENSION) = object_v_;
  x_(2 * robot_dof_ + 2 * OBJECT_DIMENSION) = in_contact;  // contact flag
  //x_.tail<TORQUE_DIMENSION>() = tau_ext_;


  //display_state();
}

void PandaRaisimDynamics::update_model()
{
  // save the current object's state
  raisim::Mat<3,1> vel_c = {0,0,0};
  raisim::Mat<3,1> pos_c = {0,0,0};
  raisim::Mat<3,3> rot_c;
  Eigen::Array<double,3,3> rot_c_e;
  Eigen::Matrix3d rotr; 
  cylinder_->getVelocity(cylinder_->getIndexInWorld(),vel_c);
  pos_c = cylinder_->getPosition();
  cylinder_->getOrientation(cylinder_->getIndexInWorld(),rot_c);
  rotr = rot_c.e();
  Eigen::AngleAxisd aa; 
  aa = rotr;
  Eigen::Quaternion<double> q;
  q=aa;

  // remove and add
  sim_.removeObject(cylinder_);
  double r = ((double) rand() / (RAND_MAX));
  double sign;
  cylinder_ = sim_.addCylinder(params_.cylinder_radius, params_.cylinder_height,
              0.5,"steel", raisim::COLLISION(1), -1); 
  cylinder_->setMass(params_.cylinder_mass);
  cylinder_->setBodyType(raisim::BodyType::DYNAMIC);
  cylinder_->setName("Cylinder");
  // pos_c[0] = pos_c[0] + (r-0.5)/50;
  // pos_c[1] = pos_c[1] + (r-0.5)/50;
  cylinder_->setPosition(pos_c);
  cylinder_->setOrientation(q);
  cylinder_->setLinearVelocity(vel_c);  //todo:  add angular velocity

  //ROS_INFO_STREAM("model updated");

}

void PandaRaisimDynamics::display_state()
{
  std::cout << " ----------------------------------- " << std::endl;
  if(if_sim_)
    std::cout << "real mug state: " << mug_->getGeneralizedCoordinate().e().transpose() << std::endl;

  if(!if_sim_)
    std::cout << "estimated primitive state:  " << cylinder_->getPosition().transpose() 
        << " , " << cylinder_->getQuaternion().transpose() << std::endl;
}

mppi::observation_t PandaRaisimDynamics::step(const mppi::input_t& u,
                                              const double dt) {                                                                                  
  set_control(u);
  advance();
  return x_;
}

void PandaRaisimDynamics::reset(const mppi::observation_t& x, const double t) {
  // internal eigen state
  t_ = t;
  x_ = x;

  // panda_->setState(x_.head<BASE_ARM_GRIPPER_DIM>(),
  //                  x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM));
  panda_->setState(x_.head(robot_dof_),
                   x_.segment(robot_dof_,robot_dof_));

  // object_->setState(x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM),
  //                   x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM + 1));
  //cylinder_->setPosition(Eigen::Vector3d(x_(2* robot_dof_),x_(2* robot_dof_+1),params_.cylinder_z));
  //cylinder_->setLinearVelocity(Eigen::Vector3d(x_(2* robot_dof_+3),x_(2* robot_dof_+4),0));
  if(if_sim_)
    mug_->setGeneralizedCoordinate({x_(2* robot_dof_),x_(2* robot_dof_+1),x_(2* robot_dof_+2),
                           x_(2* robot_dof_+3),x_(2* robot_dof_+4),x_(2* robot_dof_+5),x_(2* robot_dof_+6)});
  if(!if_sim_)
  {
    cylinder_->setPosition(x_(2* robot_dof_),x_(2* robot_dof_+1),x_(2* robot_dof_+2) + 0.5*cylinder_->getHeight());
    cylinder_->setOrientation(x_(2* robot_dof_+3),x_(2* robot_dof_+4),x_(2* robot_dof_+5),x_(2* robot_dof_+6));
  }
  table_->setPosition(Eigen::Vector3d(params_.table_position[0],params_.table_position[1],params_.table_position[2]));
}

mppi::input_t PandaRaisimDynamics::get_zero_input(
    const mppi::observation_t& x) {
  return mppi::input_t::Zero(get_input_dimension());
}

void PandaRaisimDynamics::get_end_effector_pose(
    Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
  size_t frame_id = panda_->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda_->getFramePosition(frame_id, pos);
  panda_->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}


// void PandaRaisimDynamics::get_handle_pose(Eigen::Vector3d& position,
//                                           Eigen::Quaterniond& orientation) {
//   size_t frame_id = object_->getFrameIdxByName(params_.object_handle_joint);
//   raisim::Vec<3> pos;
//   raisim::Mat<3, 3> rot;
//   object_->getFramePosition(frame_id, pos);
//   object_->getFrameOrientation(frame_id, rot);
//   position = pos.e();
//   orientation = Eigen::Quaterniond(rot.e());
// }

std::vector<force_t> PandaRaisimDynamics::get_contact_forces() {
  std::vector<force_t> forces;
  for (const auto contact : panda_->getContacts()) {
    if (contact.skip())
      continue;  /// if the contact is internal, one contact point is set to
                 /// 'skip'
    if (contact.isSelfCollision()) continue;
    force_t force;
    force.force = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() /
                  sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

void PandaRaisimDynamics::get_external_torque(Eigen::VectorXd& tau) {
  tau.setZero((int)panda_->getDOF());
  for (const auto contact : panda_->getContacts()) {
    J_contact_.setZero();
    if (!contact.skip() && !contact.isSelfCollision()) {
      panda_->getDenseJacobian(contact.getlocalBodyIndex(),
                               contact.getPosition(), J_contact_);

      // clang-format off
      // the base jacobian references the world frame, need to rotate
      // to the base reference frame
      J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), std::sin(x_(2)), 0,
                                          -std::sin(x_(2)), std::cos(x_(2)), 0,
                                          0, 0, 1;
      // transform contact to force --> to force in world frame --> to reaction torques
      tau -= J_contact_.transpose() * contact.getContactFrame().e().transpose() * contact.getImpulse().e() / sim_.getTimeStep();
      // clang-format on
    }
  }

  if (ee_force_applied_){
    J_contact_.setZero();
    panda_->getDenseFrameJacobian("panda_grasp_joint", J_contact_);

    // clang-format off
    J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
                                        std::sin(x_(2)), std::cos(x_(2)), 0,
                                        0, 0, 1;
    // clang-format on
    tau += J_contact_.transpose() * panda_->getExternalForce()[0].e();

  }
}

void PandaRaisimDynamics::get_external_wrench(Eigen::VectorXd& wrench) {
  wrench.setZero(6);
  size_t frame_id = panda_->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda_->getFramePosition(frame_id, pos);
  panda_->getFrameOrientation(frame_id, rot);

  for (const auto contact : panda_->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      // ee_frame <-- world_frame <-- force <-- impulse
      Eigen::Vector3d force_ee_frame =
          -rot.e().transpose() * contact.getContactFrame().e().transpose() *
          contact.getImpulse().e() / sim_.getTimeStep();
      Eigen::Vector3d relative_position =
          rot.e().transpose() * (contact.getPosition().e() - pos.e());
      wrench.head<3>() += force_ee_frame;
      wrench.tail<3>() = relative_position.cross(force_ee_frame);
    }
  }
  if (ee_force_applied_) {
    wrench.head<3>() += panda_->getExternalForce()[0].e();
  }
}

void PandaRaisimDynamics::get_reference_link_pose(Eigen::Vector3d& position,
                             Eigen::Quaterniond& orientation){
  size_t frame_id = panda_->getFrameIdxByName("reference_link_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda_->getFramePosition(frame_id, pos);
  panda_->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

void PandaRaisimDynamics::get_ee_jacobian(Eigen::MatrixXd& J){
  J.setZero(6, (int)panda_->getDOF());
  Eigen::MatrixXd J_linear;
  J_linear.setZero(3, 12);
  Eigen::MatrixXd J_angular;
  J_angular.setZero(3, 12);

  panda_->getDenseFrameJacobian("panda_grasp_joint", J_linear);
  panda_->getDenseFrameRotationalJacobian("panda_grasp_joint", J_angular);
  J.topRows(3) = J_linear;
  J.bottomRows(3) = J_angular;
  // clang-format off
  J.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
                             std::sin(x_(2)), std::cos(x_(2)), 0,
                             0, 0, 1;
  // clang-format on
}

void PandaRaisimDynamics::set_external_ee_force(const Eigen::Vector3d& f) {
  ee_force_applied_ = (f.norm() > 1e-4);
  auto& frame = panda_->getFrameByName("panda_grasp_joint");
  panda_->setExternalForce(frame.parentId, raisim::ArticulatedSystem::Frame::WORLD_FRAME, f, raisim::ArticulatedSystem::Frame::BODY_FRAME, raisim::Vec<3>());
}

double PandaRaisimDynamics::get_object_displacement() const {
  // return x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0);
  return x_.segment(2 * robot_dof_, OBJECT_DIMENSION)(0);
}

// void PandaRaisimDynamics::fix_object() {
//   object_->getState(object_p_, object_v_);
//   std::vector<raisim::Vec<2>> object_limits;
//   raisim::Vec<2> limit;
//   limit[0] = object_p_[0] - 0.001;
//   limit[1] = object_p_[0] + 0.001;
//   object_limits.push_back(limit);
//   object_->setJointLimits(object_limits);
// }

// void PandaRaisimDynamics::release_object() {
//   std::vector<raisim::Vec<2>> object_limits;
//   raisim::Vec<2> limit;
//   limit[0] = 0.0;
//   limit[1] = M_PI_2;
//   object_limits.push_back(limit);
//   object_->setJointLimits(object_limits);
// }

}  // namespace manipulation

