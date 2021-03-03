/*!
 * @file     sim.cpp
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#include <chrono>
#include <iostream>
#include <numeric>

#include <raisim/OgreVis.hpp>
#include <raisim/World.hpp>

#define DEFAULT_CONFIGURATION \
  0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.04, 0.04
#define VELOCITY_TARGET 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

using namespace raisim;
using namespace std::chrono;

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
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
  vis->getSceneManager()->setSkyBox(true, "Examples/StormySkyBox", 500, true,
                                    quat);
}

int main(int argc, char** argv) {
  raisim::World::setActivationKey(
      "/home/giuseppe/raisim_ws/raisimLib/build/examples/rsc/"
      "activation.raisim");

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
  auto panda = world.addArticulatedSystem(
      "/home/giuseppe/clion_ws/sampling_control_project/src/"
      "sampling_based_control/mppi_examples/"
      "mppi_manipulation/data/panda_mobile_fixed.urdf",
      "/");
  auto door = world.addArticulatedSystem(
      "/home/giuseppe/clion_ws/sampling_control_project/src/"
      "sampling_based_control/mppi_examples/"
      "mppi_manipulation/data/objects/shelf.urdf");
  auto ground = world.addGround(-1);
  // ground->setName("checkerboard"); /// not necessary here but once you set
  // name, you can later retrieve it using raisim::World::getObject()

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

  door->setGeneralizedCoordinate(doorNominalConfig);
  door->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  door->setPdGains(doorPgain, doorDgain);
  door->setPdTarget(doorNominalConfig, doorVelocityTarget);
  door->setName("door");  // this is the name assigned for raisim. Not used in
                          // this example

  /// handle
  // std::cout << "Printing handle state" << std::endl;
  // auto handle = door->getFrameByName("handle_link");
  // std::cout << handle.position << std::endl;
  // std::cout << handle.orientation << std::endl;

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
  auto panda_graphics = vis->createGraphicalObject(panda, "panda");
  auto door_graphics = vis->createGraphicalObject(door, "door");

  // anymal_gui::frame::setArticulatedSystem(anymal, 0.3); // to visualize
  // frames

  /// panda joint PD controller
  Eigen::VectorXd jointNominalConfig(9), jointVelocityTarget(9);
  Eigen::VectorXd jointState(9), jointForce(9), jointPgain(9), jointDgain(9);
  Eigen::VectorXd jointTorque(9), jointSpeed(9);

  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.setConstant(200.0);
  jointPgain(1) = 0.0;
  jointDgain.setConstant(20.0);

  /// set panda properties
  panda->setGeneralizedCoordinate({DEFAULT_CONFIGURATION});
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
  panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  panda->setPdGains(jointPgain, jointDgain);
  panda->setName("panda");  // this is the name assigned for raisim. Not used in
                            // this example

  /// panda PD controller
  jointVelocityTarget.setZero();
  jointNominalConfig << DEFAULT_CONFIGURATION;
  jointVelocityTarget << VELOCITY_TARGET;
  panda->setPdTarget(jointNominalConfig, jointVelocityTarget);

  /// contacts
  // std::vector<size_t> footIndices;
  // std::array<bool, 4> footContactState;
  // footIndices.push_back(anymal->getBodyIdx("LF_SHANK"));
  // footIndices.push_back(anymal->getBodyIdx("RF_SHANK"));
  // footIndices.push_back(anymal->getBodyIdx("LH_SHANK"));
  // footIndices.push_back(anymal->getBodyIdx("RH_SHANK"));

  /// just to get random motions of anymal
  // std::default_random_engine generator;
  // std::normal_distribution<double> distribution(0.0, 0.3);
  // std::srand(std::time(nullptr));
  // double time=0.;

  /// lambda function for the controller
  auto controller = [&world, panda, door]() {
    static double sleep_s = 1.0;
    static double time = 0;
    static int last_step = 0;
    static time_point<steady_clock> start, end;

    /// we cannot query door_frame_step since all fixed bodies are combined into
    /// one
    static size_t doorStepIndex = door->getBodyIdx("door");

    time += world.getTimeStep();
    int curr_step = (int)(time / sleep_s);
    if (curr_step > last_step) {
      std::cout << "Quering contacts for frame: " << doorStepIndex << std::endl;

      start = steady_clock::now();
      std::vector<contact::Contact> contacts = door->getContacts();
      end = steady_clock::now();

      std::cout << "There are: " << contacts.size() << " contacts."
                << std::endl;
      double query_time = duration_cast<nanoseconds>(end - start).count() / 1e6;

      for (auto& contact : door->getContacts()) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Query time is: " << query_time << " ms." << std::endl;
        if (contact.skip())
          continue;  /// if the contact is internal, one contact point is set to
                     /// 'skip'
        if (contact.isSelfCollision()) continue;
        if (contact.getlocalBodyIndex() != doorStepIndex) continue;

        /// the impulse is acting from objectB to objectA. You can check if this
        /// object is objectA or B by
        std::cout << "Door step in collision!" << std::endl;
        std::cout << "Contact impulse in the contact frame: "
                  << contact.getImpulse()->e() << std::endl;
        std::cout << "is ObjectA: " << contact.isObjectA() << std::endl;
        std::cout << "Contact frame: \n"
                  << contact.getContactFrame().e() << std::endl;
        std::cout << "Contact impulse in the world frame: "
                  << contact.getContactFrame().e() * contact.getImpulse()->e()
                  << std::endl;
        std::cout << "Contact Normal in the world frame: "
                  << contact.getNormal().e().transpose() << std::endl;
        std::cout << "Contact position in the world frame: "
                  << contact.getPosition().e().transpose() << std::endl;
        std::cout << "It collides with: "
                  << world.getObject(contact.getPairObjectIndex())->getName()
                  << std::endl;
        std::cout << "please check Contact.hpp for the full list of the methods"
                  << std::endl;
      }

      last_step = curr_step;
    }
  };

  // auto controller = [anymal,
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
  //    anymal->setGeneralizedCoordinate({0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03,
  //    0.4,
  //                                      -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
  //                                      0.8, -0.03, -0.4, 0.8});
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
    jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.3, -.6, -0.03, 0.3, -.6,
  0.03, -0.3, .6, -0.03, -0.3, .6;

    for (size_t k = 0; k < anymal->getGeneralizedCoordinateDim(); k++)
      jointNominalConfig(k) += distribution(generator);

    anymal->setPdTarget(jointNominalConfig, jointVelocityTarget);

    /// check if the feet are in contact with the ground
    for(auto& fs: footContactState) fs = false;

    for(auto& contact: anymal->getContacts()) {
      auto it = std::find(footIndices.begin(), footIndices.end(),
  contact.getlocalBodyIndex()); size_t index = it - footIndices.begin(); if
  (index < 4) footContactState[index] = true;
    }

    /// torque, speed and contact state
    anymal_gui::joint_speed_and_torque::push_back(time, jointSpeed,
  jointTorque); anymal_gui::gait::push_back(footContactState);

    /// just displaying random numbers since we are not doing any training here
    anymal_gui::reward::log("torque", distribution(generator));
    anymal_gui::reward::log("commandTracking", distribution(generator));
  };

*/
  /// sim callback
  vis->setControlCallback(controller);

  /// set camera
  vis->select(door_graphics->at(0), false);
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4),
                                       2);

  /// run the app
  vis->run();

  /// terminate
  vis->closeApp();

  return 0;
}
