//
// Created by giuseppe on 10.02.21.
//
// debug node to visualize the state of panda

#include <chrono>
#include <iostream>
#include <numeric>

#include <raisim/OgreVis.hpp>
#include <raisim/World.hpp>

#include <manipulation_msgs/State.h>
#include <manipulation_msgs/conversions.h>
#include <mppi_manipulation/dynamics.h>

#include <ros/ros.h>

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
  ros::init(argc, argv, "raisim_ogre_rendering_node");
  ros::NodeHandle nh("~");

  /// Initialize the dynamics
  std::string robot_description_raisim;
  if (!nh.getParam("/robot_description_ogre", robot_description_raisim)) {
    ROS_ERROR(
        "Could not parse robot_description_raisim. Is the parameter set?");
    return 0;
  }

  std::string object_description_raisim;
  if (!nh.getParam("/object_description_raisim", object_description_raisim)) {
    ROS_ERROR(
        "Could not parse object_description_raisim. Is the parameter set?");
    return 0;
  }

  bool fixed_base;
  if (!nh.getParam("fixed_base", fixed_base)) {
    ROS_ERROR("Could not parse fixed_base. Is the parameter set?");
    return 0;
  }

  manipulation::PandaRaisimDynamics dynamics(
      robot_description_raisim, object_description_raisim, 0.01, fixed_base);

  /// just a shortcut
  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  dynamics.get_world()->setTimeStep(0.0001);
  vis->setWorld(dynamics.get_world());
  vis->setWindowSize(1800, 1200);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);

  /// starts visualizer thread
  vis->initApp();

  /// create additional raisim objects
  auto ground = dynamics.get_world()->addGround(-0.2);
  ground->setName(
      "checkerboard");  /// not necessary here but once you set name, you can
                        /// later retrieve it using raisim::World::getObject()

  /// create visualizer objects
  auto ground_graphics =
      vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
  auto object_graphics =
      vis->createGraphicalObject(dynamics.get_object(), "object");
  auto panda_graphics =
      vis->createGraphicalObject(dynamics.get_panda(), "panda");

  mppi::DynamicsBase::observation_t current_state;

  auto cb = [&](const manipulation_msgs::StateConstPtr& msg) {
    manipulation::conversions::msgToEigen(*msg, current_state);
    dynamics.reset(current_state);
  };

  ros::Subscriber current_state_subscriber =
      nh.subscribe<manipulation_msgs::State>("/observer/state", 1, cb);

  /// lambda function for the controller
  auto controller = []() { ros::spinOnce(); };

  /// sim callback
  vis->setControlCallback(controller);

  /// set camera
  vis->select(panda_graphics->at(0), false);
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4),
                                       2);

  /// run the app
  try {
    vis->run();
  } catch (const ros::Exception& exc) {
    ROS_INFO_STREAM(exc.what());
  }

  /// terminate
  vis->closeApp();
  return 0;
}
