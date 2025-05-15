# Installation and Compilation Guide

English|[cn中文](./README_zh.md)



## Basic Environment Setup

### 1. ROS2-jazzy

#### Installation

##### Method 1

```bash
wget http://fishros.com/install -O fishros && . fishros
```

##### Method 2

[Official ROS2-jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

#### Verification

Open **Terminal 1**, and enter the following command:

```bash
ros2 run demo_nodes_cpp talker
```

The node starts publishing messages, as shown below:

<img src="attachments/image-20250515100124847.png" style="zoom:50%;" alt="talker_node" align=center/>

Open **Terminal 2**, and enter the following command:

```bash
ros2 run demo_nodes_cpp listener
```

If **Terminal 1** is still open, **Terminal 2** will receive messages published by the `talker` node, as shown below:

<img src="attachments/image-20250515100500405.png" style="zoom:50%;" alt="listener_node" align=center/>

**ROS2 installation successful!**

#### Note for WSL2 Installation

If you are installing ROS2 on **WSL2** with the network mode set to **mirror mode**, it may affect ROS2 message passing, causing the `listener` node to fail to receive messages.

##### Solution 1

Before running the `listener` node, manually enter `ros2 daemon start` to start the ROS2 daemon, which remains active until the system terminates.

##### Solution 2

Add the command to the `.bashrc` file to run it automatically when the terminal starts:

```bash
echo "ros2 daemon start" >> ~/.bashrc
source ~/.bashrc
```

### 2. Unitree-ros2

If the Unitree-ros2 package is not installed, you will encounter the following error during compilation:

<img src="attachments/image-20250515145701666.png" style="zoom:50%;" alt="fault 1" align=center/>

#### Installation

In the terminal, enter the following commands:

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
sudo apt install ros-jazzy-rosidl-generator-dds-idl
```

Before compiling cyclonedds, ensure that the ROS2 environment variables are not automatically sourced when starting the terminal. If you added "source /opt/ros/jazzy/setup.bash" to the ~/.bashrc file during ROS2 installation, modify the ~/.bashrc file to comment it out:

```bash
# source /opt/ros/jazzy/setup.bash
# ros2 daemon start
```

Open a new terminal and enter the following commands to compile cyclone-dds:

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b jazzy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

After compiling cyclone-dds, ROS2 dependencies are required to complete the compilation of the Unitree package. Therefore, source the ROS2 environment variables before compiling:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

Modify the ROS version in the `~/unitree_ros2/setup.sh`与`~/unitree_ros2/setup_local.sh` from `foxy` to `jazzy` and activate the environment for subsequent compilations:

```bash
source ~/unitree_ros2/setup.sh  # Run when connecting to the physical robot
source ~/unitree_ros2/setup_local.sh  # Run when using simulation only
```

### 3. Source Code Download

Create a workspace and download the source code:

```bash
cd ~
mkdir unitree-mpc && cd unitree-mpc
git clone https://github.com/sm-1z/unitree_mpc.git
```

The downloaded unitree_mpc includes the wb_humanoid_mpc and unitree_mujoco packages. Move these packages to the `src` directory under the root directory `unitree-mpc`, or directly rename the unitree_mpc directory to `src`:

```bash
mv unitree_mpc src
```

The final directory structure should look like this:

```
unitree_mpc
	-src
		-wb_humanoid_mpc
		-unitree_mujoco
```

### 4. Install Dependencies

Install dependencies based on the local dependency file. In the terminal, enter the following command to install dependencies:

```bash
cd src/wb_humanoid_mpc/
envsubst < dependencies.txt | xargs sudo apt install -y
```

## Compile the Source Code

#### Compiling wb_humanoid_mpc

Before compiling, ensure that the following command is run in the terminal based on specific conditions:

```bash
source ~/unitree_ros2/setup.sh  # Run when connecting to the physical robot
source ~/unitree_ros2/setup_local.sh  # Run when using simulation only
```

Run the following command to compile in the terminal, located in the `unitree-mpc/src/wb_humanoid_mpc/` directory:

```bash
make build-all
```

#### Compiling unitree_mujoco

To compile `unitree_mujoco`, follow these steps:

```bash
source ~/unitree-mpc/install/setup.bash
```

**Navigate to the Build Directory**: Change to the `simulate` directory within `unitree_mujoco`:

```bash
cd ~/unitree-mpc/src/unitree_mujoco/simulate
```

**Create and Enter the Build Directory**: Create a `build` directory and navigate into it:

```bash
mkdir build && cd build
```

**Run CMake**: Configure the project using CMake:

```bash
cmake ..
```

**Compile the Project**: Compile the project using `make`:

```bash
make
```

#### Note for Conda Environment

The Makefile specifies the location of the Python interpreter, which may not match the default Conda Python interpreter location. If an error occurs, you can refer to and modify the Makefile accordingly.

## Run

After compiling, enter the following command in the terminal to run. Located in the `unitree-mpc/src/wb_humanoid_mpc/` directory:

Once you run the NMPC, a window with Rviz will appear for visualization. The first time you start MPC for a robot model, the automatic differentiation code will be generated, which may take 5 to 15 minutes, depending on your system. Once completed, the robot will appear, and you can control it using an Xbox game controller or commands in the terminal.

For **Center of Mass Dynamics Model Predictive Control**:

```bash
make launch-g1-dummy-sim
```

For **Whole-Body Dynamics Model Predictive Control**:

```bash
make launch-wb-g1-dummy-sim
```

Run mujoco

```bash
source ~/unitree-mpc/install/setup.bash
cd ~/unitree-mpc/src/unitree_mujoco/simulate/build
./unitree_mujoco
```



## References

[ubuntu - wsl2使用镜像模式导致ROS2的通讯不可用? - SegmentFault 思否](https://segmentfault.com/q/1010000046011866)

[unitree_ros2/README _zh.md at master · unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/blob/master/README _zh.md)

