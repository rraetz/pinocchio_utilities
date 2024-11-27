# Purpose
This is a collection of utility functions to be used with the wonderful [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) library for robotics.

# Dependencies
### Pinocchio
On an x86_64 platform, you can install Pinocchio as indicated [here](https://stack-of-tasks.github.io/pinocchio/download.html). Below are instructions for non-x86_64 platforms. The easiest is to install it through ROS2 as the binaries are already available.

Urdfdom:
```
sudo apt-get install liburdfdom-dev liburdfdom-headers-dev libtinyxml2-dev
```
Eigen:
```
sudo apt-get install libeigen3-dev
```
Boost:
```
sudo apt-get install libboost-filesystem-dev libboost-serialization-dev libboost-system-dev
```

ROS2 already provides Pinocchio binaries for different platforms, so let's take advantage of this. First make the ROS packages available (instructions take from [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)):
```bash
# Install dependencies
sudo apt install software-properties-common
sudo add-apt-repository universe
# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add repo to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Update apt list
sudo apt update
```

Then install Pinocchio:
```
sudo apt-get install ros-humble-pinochio
```

### Meshcat-Cpp
Meshcat is a tool to visualize robots in the browser. It is not necessary for the collision checking, but it is a nice tool to have. This dependency is directly resolved in the CMakeLists.txt file as it needs to be compiled.


### Plog
The example uses Plog as logging library. This dependency is also directly resolved in the CMakeLists.txt.

# Example Usage
The /src folder contains a simple example of how to use Pinocchio to run collision checking of a robot with a static environment. Build using CMake: 
```
mkdir build && cd build
cmake ..
make
```
Run the example (from inside the root folder):
```
./build/pinocchio_utilities_example
```
Connect to http://127.0.0.1:7001/ for the Meshcat visualization.
