# Instructions

## Install dependencies:
The installation of Pinocchio can be a bit tricky, especially on non-x86_64 platforms. Here are instructions that should also build on aarch64.
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

## Example Usage
The /src folder contains a simple example of how to use Pinocchio to run collision checking of a robot with a static environment. Build using CMake and run the executable. Also check the CMakeLists.txt file for the necessary dependencies.
