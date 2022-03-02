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
    ROS_INFO_STREAM( "this dynamics is as [REAL WORLD] dynamics");
  }
  if(!if_sim_)
  {
    ROS_INFO_STREAM("this dynamics is as [MODEL ESTIMATED] dynamics");
    
    if_update_ = params_.update_geometry;

    if(if_update_)
      ROS_INFO_STREAM("activate model geometry updater ");

  }
  
  initialize_world(params_.robot_description, 
                  params_.mug_description);
  initialize_pd();
  set_collision();

  t_ = 0.0;
  ee_force_applied_ = false;

};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    std::vector<std::string>& mug_description) {

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

  //  create cylinders/primitives
  if(!if_sim_)
  {
    cylinder_.resize(OBJECT_NUMBER);
    // [Debug Feb. 2]: open in 2nd round
    for(int i = 0 ; i < OBJECT_NUMBER; i++)
    {
      cylinder_[i] = sim_.addCylinder(params_.cylinder_radius[i], params_.cylinder_height[i],
                  0.5,"steel", raisim::COLLISION(1), -1); 
      cylinder_[i]->setMass(params_.cylinder_mass[i]);
      cylinder_[i]->setBodyType(raisim::BodyType::DYNAMIC);
      cylinder_[i]->setName("cylinder_" + std::to_string(i));
    }
  }

  // create mugs 
  if(if_sim_)
  { 
    mug_.resize(OBJECT_NUMBER);
    for(int i = 0 ; i < OBJECT_NUMBER; i ++)
    {
      mug_description_ = mug_description[i];
      mug_[i] = sim_.addArticulatedSystem(mug_description_, "/",{},
                        raisim::COLLISION(1), -1);
    }
  }

  // To simulate 2D sliding, we need a table 
  table_ = sim_.addBox(1.5,1.5,0.1,10,"steel",
          raisim::COLLISION(1), -1);
  table_->setBodyType(raisim::BodyType::STATIC); //no velocity, inf mass
  table_->setName("Table");
  table_->setPosition(Eigen::Vector3d(params_.table_position[0],params_.table_position[1],params_.table_position[2]));
  
  // state size init, according to DOF
  robot_dof_ = BASE_ARM_GRIPPER_DIM;
  obj_dof_ = OBJECT_DIMENSION;
  obj_num_ = OBJECT_NUMBER;
  state_dimension_ = STATE_DIMENSION;
  input_dimension_ = INPUT_DIMENSION;
  x_ = mppi::observation_t::Zero(STATE_DIMENSION);
  
  // init state vars
  object_p_.setZero(OBJECT_NUMBER * OBJECT_DIMENSION);
  object_v_.setZero(OBJECT_NUMBER * OBJECT_DIMENSION);

  // init state setup
  reset(params_.initial_state, t_);

  ROS_INFO_STREAM("table inited at: " << table_->getPosition().transpose() ); 

  for(int i = 0 ; i < OBJECT_NUMBER; i ++)
  {
    if (if_sim_)
      ROS_INFO_STREAM( "mug [" << i << "] inited at: " << mug_[i]->getGeneralizedCoordinate() ); 
    if (!if_sim_)
      ROS_INFO_STREAM( "cylinder [" << i << "]inited at: " << cylinder_[i]->getPosition().transpose() ); 
  }
}

void PandaRaisimDynamics::initialize_pd() {
  /// panda
  cmd_.setZero(BASE_ARM_GRIPPER_DIM);
  cmdv_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_p_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_v_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_p_gain_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_d_gain_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_p_desired_.setZero(BASE_ARM_GRIPPER_DIM);
  joint_v_desired_.setZero(BASE_ARM_GRIPPER_DIM);

  // clang-format off
  joint_p_gain_.segment(BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kp;
  joint_d_gain_.segment(BASE_DIMENSION, ARM_DIMENSION) = params_.gains.arm_gains.Kd;
  joint_p_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kp;
  joint_d_gain_.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kd;

  // clang-format on

  panda_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);\
  panda_->setPdGains(joint_p_gain_, joint_d_gain_);
  panda_->setGeneralizedForce(Eigen::VectorXd::Zero(panda_->getDOF()));

}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : panda_->getBodyNames())
  {
    pandaBodyIdxs.push_back(panda_->getBodyIdx(bodyName));
    ROS_INFO_STREAM("panda body idx: " << panda_->getBodyIdx(bodyName));
  }
  if(!if_sim_)
  {
  ROS_INFO_STREAM("cylinder idx: " << cylinder_[0]->getIndexInWorld());
  ROS_INFO_STREAM("cylinder idx: " << cylinder_[1]->getIndexInWorld());
  }
  ROS_INFO_STREAM("panda idx: " << panda_->getIndexInWorld());
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      panda_->ignoreCollisionBetween(body_idx1, body_idx2);
}

void PandaRaisimDynamics::set_control(const mppi::input_t& u) {
  // keep the gripper in the current position

  cmd_.tail<PandaDim::GRIPPER_DIMENSION>()
      << x_.head<BASE_ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();

  // arm
  cmdv_.segment<ARM_DIMENSION>(BASE_DIMENSION) = u.segment<ARM_DIMENSION>(BASE_DIMENSION);

  if(if_sim_)
  {}

  if(!if_sim_)
  {
      //ROS_INFO_STREAM("primitive input: " << u.transpose());  
  }

  // gripper
  cmdv_.tail<PandaDim::GRIPPER_DIMENSION>().setZero();
  
  // PD low-level controller
  panda_->setPdTarget(cmd_, cmdv_);
  panda_->setGeneralizedForce(panda_->getNonlinearities(gravity_));
  
}

void PandaRaisimDynamics::advance() {
  
  int in_contact = 0;
  auto panda_idx = panda_->getIndexInWorld();

  if(if_sim_)
  { 
    // get contact state
    for (const auto& contact : mug_[0]->getContacts()) {
      if (!contact.skip() && !contact.isSelfCollision() 
          && (sim_.getObject(contact.getPairObjectIndex())->getIndexInWorld() == panda_idx) ) 
      { 
        // std::cout << " in contact with panda " << std::endl;
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

    for(int i = 0 ; i < OBJECT_NUMBER; i ++)
    {
      // update object state
      object_p_.segment<OBJECT_DIMENSION>(OBJECT_DIMENSION * i) = mug_[i]->getGeneralizedCoordinate().e().segment<7>(0);
      raisim::Vec<3> obj_vel;
      mug_[i]->getVelocity(0, obj_vel);  // TODO:: use get idx to replace here
      object_v_.segment<3>(OBJECT_DIMENSION * i) = obj_vel.e();
      // //TODO: update real trans and rot vel here

      object_v_(4 + OBJECT_DIMENSION * i) = params_.cylinder_radius[i];
      object_v_(5 + OBJECT_DIMENSION * i) = params_.cylinder_height[i];
    }
    // mug_->setExternalForce(0, {1.5,0,0});

  }

  if(!if_sim_)
  { 
    // get contact state
    for (const auto& contact : cylinder_[1]->getContacts()) {
      if (!contact.skip() && !contact.isSelfCollision() 
          && (sim_.getObject(contact.getPairObjectIndex())->getIndexInWorld() == panda_idx) ) 
      { 
        //ROS_INFO_STREAM( " in contact with panda ");
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
    
    for( int i = 0 ; i < OBJECT_NUMBER; i ++)
    {
      // [Debug Feb. 2]: open in 3rd round
      object_p_.segment<3>(OBJECT_DIMENSION * i) = cylinder_[i]->getPosition();
      
      // [Debug Feb. 2]: open in 4th round
      object_p_.segment<4>(OBJECT_DIMENSION * i + 3) = cylinder_[i]->getQuaternion();
      object_v_.segment<3>(OBJECT_DIMENSION * i) = cylinder_[i]->getLinearVelocity();
      // TODO: currently use velocity to pass geometry info, correct this after the experiment 

      // [Debug Feb. 2]: open in 3rd round
      object_v_(4 + OBJECT_DIMENSION * i) = cylinder_[i]->getRadius();
      object_v_(5 + OBJECT_DIMENSION * i) = cylinder_[i]->getHeight();

    }


  }

  x_.head<BASE_ARM_GRIPPER_DIM>() = joint_p_;
  x_.segment(BASE_ARM_GRIPPER_DIM, BASE_ARM_GRIPPER_DIM) = joint_v_;
  x_.segment(2 * BASE_ARM_GRIPPER_DIM, OBJECT_DIMENSION * OBJECT_NUMBER) = object_p_;
  x_.segment(2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION * OBJECT_NUMBER, OBJECT_DIMENSION * OBJECT_NUMBER) = object_v_;
  x_(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION * OBJECT_NUMBER) = in_contact;  // contact flag

  // display_state();

}

// [Debug Feb. 2]: Never get called in all rounds
void PandaRaisimDynamics::update_geometry()
{

  // save the current object's state
  // raisim::Mat<3,1> vel_c = {0,0,0};
  // raisim::Mat<3,1> pos_c = {0,0,0};
  // raisim::Mat<3,3> rot_c;
  // Eigen::Array<double,3,3> rot_c_e;
  // Eigen::Matrix3d rotr; 
  // cylinder_->getVelocity(cylinder_->getIndexInWorld(),vel_c);
  // vel_c = cylinder_->getLinearVelocity();
  // //pos_c = cylinder_->getPosition();
  // pos_c = x_.segment<3>(2* BASE_ARM_GRIPPER_DIM);
  // cylinder_->getOrientation(cylinder_->getIndexInWorld(),rot_c);
  // rotr = rot_c.e();
  // Eigen::AngleAxisd aa; 
  // aa = rotr;
  // Eigen::Quaternion<double> q;
  // //q=aa;
  // q.x() = x_(2* BASE_ARM_GRIPPER_DIM+3);
  // q.y() = x_(2* BASE_ARM_GRIPPER_DIM+4);
  // q.z() = x_(2* BASE_ARM_GRIPPER_DIM+5);
  // q.w() = x_(2* BASE_ARM_GRIPPER_DIM+6);

  // remove and add
  for(int i = 0 ; i < OBJECT_NUMBER; i ++)
  {
    sim_.removeObject(cylinder_[i]);
    cylinder_[i] = sim_.addCylinder(x_(2 * BASE_ARM_GRIPPER_DIM + i*OBJECT_DIMENSION + OBJECT_DIMENSION * OBJECT_DIMENSION +4), 
                                    x_(2 * BASE_ARM_GRIPPER_DIM + i*OBJECT_DIMENSION + OBJECT_DIMENSION * OBJECT_DIMENSION +5), 
                                    0.5,"steel", raisim::COLLISION(1), -1); 

    //set the geometry state
    cylinder_[i]->setMass(params_.cylinder_mass[i]);
    cylinder_[i]->setBodyType(raisim::BodyType::DYNAMIC);
    cylinder_[i]->setName("Cylinder");
  }
  // ROS_INFO_STREAM("geometry updated");

}

Eigen::VectorXd PandaRaisimDynamics::get_primitive_state(){
    Eigen::VectorXd object_pv;
    object_pv.setZero(OBJECT_DIMENSION*2*OBJECT_NUMBER);
    object_pv << object_p_, object_v_;
    return object_pv;
}


void PandaRaisimDynamics::display_state()
{ 
  std::cout << " ----------------------------------- " << std::endl;
  
  auto obj_num = sim_.getConfigurationNumber();
  ROS_INFO_STREAM("num of object in raisim: " << obj_num);

  if(if_sim_)
    ROS_INFO_STREAM( "real mug state: " << mug_[0]->getGeneralizedCoordinate().e().transpose() );

  if(!if_sim_)
    ROS_INFO_STREAM( "estimated primitive state:  " << cylinder_[0]->getPosition().transpose() 
        << " , " << cylinder_[0]->getQuaternion().transpose() );

  ROS_INFO_STREAM("contact number: " << cylinder_[0]->getContacts().size()); 
  
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

  panda_->setState(x_.head(BASE_ARM_GRIPPER_DIM),
                   x_.segment(BASE_ARM_GRIPPER_DIM,BASE_ARM_GRIPPER_DIM));
  if(if_sim_)
  {
    for(int i = 0 ; i < OBJECT_NUMBER; i ++)
    { 
      mug_[i]->setGeneralizedCoordinate(x_.segment<7>(2* BASE_ARM_GRIPPER_DIM + i*OBJECT_DIMENSION));
    }
  }
   
  if(!if_sim_)
  { 
    if(if_update_)     // update model depending on the flag
    {
      update_geometry();
    }
    // [Debug Feb. 2]: open in 2nd round
    for(int i = 0 ; i < OBJECT_NUMBER; i ++)
    {  
      cylinder_[i]->setPosition(Eigen::Vector3d(x_.segment<3>(2* BASE_ARM_GRIPPER_DIM + i*OBJECT_DIMENSION)));
      cylinder_[i]->setOrientation(Eigen::Vector4d(x_.segment<4>(2* BASE_ARM_GRIPPER_DIM + i* OBJECT_DIMENSION + 3)));
      cylinder_[i]->setLinearVelocity(x_.segment<3>(2* BASE_ARM_GRIPPER_DIM+OBJECT_DIMENSION*OBJECT_NUMBER + i* OBJECT_DIMENSION ));
    }
  }

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
  return x_.segment(2 * BASE_ARM_GRIPPER_DIM, OBJECT_DIMENSION)(0);
}

}  // namespace manipulation

