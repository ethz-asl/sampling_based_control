/*!
 * @file     sim.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include <iostream>
#include <chrono>
#include <numeric>

#include <raisim/OgreVis.hpp>
#include <raisim/World.hpp>

#define DEFAULT_CONFIGURATION 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.04, 0.04

using namespace raisim;

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3,-0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// show setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(10);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.03, 0.6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);

  /// skybox
  Ogre::Quaternion quat;
  quat.FromAngleAxis(Ogre::Radian(M_PI_2), {1., 0, 0});
  vis->getSceneManager()->setSkyBox(true,
                                    "Examples/StormySkyBox",
                                    500,
                                    true,
                                    quat);
}

int main(int argc, char **argv) {
  raisim::World::setActivationKey("/home/giuseppe/raisim_ws/raisimlib/rsc/activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.0025);

  /// just a shortcut
  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(1800, 1200);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);

  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto panda = world.addArticulatedSystem("/home/giuseppe/clion_ws/sampling_based_control_project/src/sampling_based_control/mppi_examples/mppi_panda_raisim/data/panda.urdf", "/");
  auto door = world.addArticulatedSystem("/home/giuseppe/clion_ws/sampling_based_control_project/src/sampling_based_control/mppi_examples/mppi_panda_raisim/data/door.urdf");
  auto ground = world.addGround(-1);
  //ground->setName("checkerboard"); /// not necessary here but once you set name, you can later retrieve it using raisim::World::getObject()

  /// set door placement and control
  /// panda joint PD controller
  Eigen::VectorXd doorNominalConfig(1), doorVelocityTarget(1);
  Eigen::VectorXd doorState(1), doorForce(1), doorPgain(1), doorDgain(1);
  Eigen::VectorXd doorTorque(1), doorSpeed(1);

  doorPgain.setZero();
  doorDgain.setZero();
  doorVelocityTarget.setZero();
  doorNominalConfig.setZero();
  doorPgain.setConstant(20.0);
  doorDgain.setConstant(1.0);

  door->setBasePos(Eigen::Vector3d(0.85, 0.0, 0.01));
  door->setGeneralizedCoordinate(doorNominalConfig);
  door->setBaseOrientation(Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));
  door->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  door->setPdGains(doorPgain, doorDgain);
  door->setPdTarget(doorNominalConfig, doorVelocityTarget);
  door->setName("door"); // this is the name assigned for raisim. Not used in this example

  /// handle
  std::cout << "Printing handle state" << std::endl;
  auto handle = door->getFrameByName("handle_link");
  std::cout << handle.position << std::endl;
  std::cout << handle.orientation << std::endl;

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
  auto panda_graphics = vis->createGraphicalObject(panda, "panda");
  auto door_graphics = vis->createGraphicalObject(door, "door");

  //anymal_gui::frame::setArticulatedSystem(anymal, 0.3); // to visualize frames

  /// panda joint PD controller
  Eigen::VectorXd jointNominalConfig(9), jointVelocityTarget(9);
  Eigen::VectorXd jointState(9), jointForce(9), jointPgain(9), jointDgain(9);
  Eigen::VectorXd jointTorque(9), jointSpeed(9);

  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.setConstant(200.0);
  jointDgain.setConstant(1.0);

  /// set panda properties
  panda->setGeneralizedCoordinate({DEFAULT_CONFIGURATION});
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
  panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  panda->setPdGains(jointPgain, jointDgain);
  panda->setName("panda"); // this is the name assigned for raisim. Not used in this example

  /// panda PD controller
  jointVelocityTarget.setZero();
  jointNominalConfig << DEFAULT_CONFIGURATION;
  panda->setPdTarget(jointNominalConfig, jointVelocityTarget);

  /// contacts
  //std::vector<size_t> footIndices;
  //std::array<bool, 4> footContactState;
  //footIndices.push_back(anymal->getBodyIdx("LF_SHANK"));
  //footIndices.push_back(anymal->getBodyIdx("RF_SHANK"));
  //footIndices.push_back(anymal->getBodyIdx("LH_SHANK"));
  //footIndices.push_back(anymal->getBodyIdx("RH_SHANK"));

  /// just to get random motions of anymal
  //std::default_random_engine generator;
  //std::normal_distribution<double> distribution(0.0, 0.3);
  //std::srand(std::time(nullptr));
  //double time=0.;

  /// lambda function for the controller
  //auto controller = [anymal,
  //    &generator,
  //    &distribution,
  //    &jointTorque,
  //    &jointSpeed,
  //    &time,
  //    &world,
  //    &footIndices,
  //    &footContactState]() {
  //  static size_t controlDecimation = 0;
  //  time += world.getTimeStep();

  //  if(controlDecimation++ % 2500 == 0) {
  //    anymal->setGeneralizedCoordinate({0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
  //                                      -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  //    raisim::anymal_gui::reward::clear();
  //    raisim::anymal_gui::gait::clear();
  //    raisim::anymal_gui::joint_speed_and_torque::clear();

  //    time = 0.;
  //  }

  /*
    jointTorque = anymal->getGeneralizedForce().e().tail(12);
    jointSpeed = anymal->getGeneralizedVelocity().e().tail(12);

    if(controlDecimation % 100 != 0)
      return;

    /// ANYmal joint PD controller
    Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.3, -.6, -0.03, 0.3, -.6, 0.03, -0.3, .6, -0.03, -0.3, .6;

    for (size_t k = 0; k < anymal->getGeneralizedCoordinateDim(); k++)
      jointNominalConfig(k) += distribution(generator);

    anymal->setPdTarget(jointNominalConfig, jointVelocityTarget);

    /// check if the feet are in contact with the ground
    for(auto& fs: footContactState) fs = false;

    for(auto& contact: anymal->getContacts()) {
      auto it = std::find(footIndices.begin(), footIndices.end(), contact.getlocalBodyIndex());
      size_t index = it - footIndices.begin();
      if (index < 4)
        footContactState[index] = true;
    }

    /// torque, speed and contact state
    anymal_gui::joint_speed_and_torque::push_back(time, jointSpeed, jointTorque);
    anymal_gui::gait::push_back(footContactState);

    /// just displaying random numbers since we are not doing any training here
    anymal_gui::reward::log("torque", distribution(generator));
    anymal_gui::reward::log("commandTracking", distribution(generator));
  };

  vis->setControlCallback(controller);
*/
  /// set camera
  vis->select(panda_graphics->at(0), false);
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

  /// run the app
  vis->run();

  /// terminate
  vis->closeApp();

  return 0;
}
