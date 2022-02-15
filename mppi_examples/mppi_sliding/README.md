# mppi_sliding

This is a closed loop control example using mppi controller, both royal panda and panda can be used as a platform. 

Current task: add a control interface for real panda manipulation test.

- Finished:
    - Modify **lanch/robot.launch** to start the real control process.
    - The controller class is named as **ManipulationController**.
    - Modify **CmakeList**, **controller_mug.yaml**, add **plugins.xml** accordingly.
    - The controller is developed in **real_controller.cpp**. 
    - In the controller, remove safety filter and energy tank. Modify state dimension, remove base-related updating and publishing functions. 

- Unfinished:
    - Intergrate the perception part into the controller, update the object's state from the keypoint tracker. 
        - Build connection between kp tracker and primitive approximation node.
        - Publish primitive object's topic to the controller, use the input state as the observation. 
        - Params tuning.
        - ...

    - Check if all params are parsed correctly.

- Unclear:
    - Not sure if **franka_gripper.launch** is needed.
    - Use torque control, or velociry control? (for now is torque control)
        -If use vel control, need to modify trasmission definition? 
    - In the controller_mug.yaml, what should be the correct namespace for PID controller of Panda?  

