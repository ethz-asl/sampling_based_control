# Policy Learning
This package contains most of the code for the PLR project *Learn How to Explore*. The goal of the project is to use imitation learning offline to train a NN (the policy). The policy tries to imitate a version of the MPPI controller that can do much more costly optimizations because it is used offline, thus no real-time capabilities are needed. The learned policy is then in the normal MPPI controller (the one that can run real-time) to generate a part of the rollouts. This allows to inject the better solutions found beforehand to the real-time system. 

**!! Currently, only `mppi_panda` is supported. !!**

## Installation
### LibTorch
Install libtorch with CXX11 ABI (this is not the default install that comes with pytorch). This is needed so that the packages using libtorch can be linked with ROS. The default pytorch installation uses the pre-CXX11 ABI.
- download distribution from [here](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.8.1%2Bcpu.zip)
- unzip to folder `<LIBTORCH_PATH>`, e.g. `/home/<USER>/libtorch/`
- add the following line to your `~/.bashrc`: `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:<LIBTORCH_PATH>`

### High Five (C++ HDF5 Library)
- Check if libhdf5-dev is installed with `apt list | grep libhdf5-dev`. If not, install it with `sudo apt-get install libhdf5-dev`.
- Clone repository from [here](https://github.com/BlueBrain/HighFive) to folder `<HIGH_FIVE>`, e.g. `/home/<USER>/HighFive`
- Install via CMAKE to folder `<HIGH_FIVE_INSTALL>`, e.g. `/home/<USER>/HighFiveBuild` by first going in folder `<HIGH_FIVE>` and calling
```
mkdir build && cd build
# Look up HighFive CMake options, consider inspecting with `ccmake`. These options should be ok for us:
cmake .. -DHIGHFIVE_USE_EIGEN=ON -DHIGHFIVE_USE_BOOST=OFF -DHIGHFIVE_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="<HIGH_FIVE_INSTALL>"
make install
```
- Add the following line to your `~/.bashrc`: `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:<HIGH_FIVE_INSTALL>`

### Python libraries

See  `requirements.txt`. A lot of the listed packages are needed to use rospy in a virtual environment. If you don't care about virtual environments it should be sufficient to install ROS (noetic), pytorch and h5py. 

## Run with trained policy

`roslaunch mppi_panda panda_control.launch learner_output_path:=<PATH> torchscript_model_path:=<PATH>`
Leaving out one of two paths will disable the repective function (either no output or no loaded model). Make sure that the parameter `learned_rollout_ratio` is set to a positive value, e.g. 0.1. 

## Train new Policy with DAGGER
Tune the MPPI parameters to get the desired performance, this will be the privileged expert that the policy will learn to imitate. Set the desired parameters for the training in `dagger.py` and if needed adapt the network architecture in `learning.py`.

In one terminal:
`roslaunch mppi_panda panda_dagger.launch`

In another terminal:
`roscd policy_learning; cd scripts; python3 dagger.py`

## Evaluate the Controllers
Set the desired MPPI parameters for the controller you want to evaluate. Adapt the parameters in `evaluate.py`, e.g. to select a policy or to specify a list of start-end poses.

In one terminal:
`roslaunch mppi_panda panda_dagger.launch`

In another terminal:
`roscd policy_learning; cd scripts; python3 evaluate.py`
