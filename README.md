
# 2023 Submarine Capstone
This GitHub repo will go over the entire process of making a submersible ROV using custom PCBs, and almost entirely 3D printed hull pieces. This project was borne from ideas pooled by Dyllon Dunton and Jacob Wildes to fulfill a senior Capstone at the University of Maine. The end goal of this project is to have a submersible drone which can be operated by human input, and all relevent telemetry (i.e. forward velocity, and depth) will be displayed to the user. Additionally, as Jacob progresses with his undergrad thesis work, eventually the need for human input will dwindle. Instead, a neural network will randomly patrol an area and as things appear (i.e. fish, lakegrass, etc.) that object will be outlined, and if desired, saved to local memory. By using IMU telemetry, the submarine should not continuously patrol the same route, though currents and other unconsidered elements may alter this. However, that is a non-issue because of the nature of the environment. Aquatic life is not constrained to the same location at all times, so even if it were in the same area twice, there may be something there that was not before.


## Index
[STL Docs](https://github.com/jacobcwildes/Submarine_Capstone/tree/main/manufacturing_stls/README.md)

[PCB Docs](https://github.com/jacobcwildes/Submarine_Capstone/blob/main/pcb_files/README.md)

[Node Docs](https://github.com/jacobcwildes/Submarine_Capstone/tree/main/ros2_ws)

[STM Docs](https://github.com/jacobcwildes/Submarine_Capstone/tree/main/stm32_ws)

## Software Requirements
- Machine must be running Ubuntu 20.04.5 LTS
- ROS2 Foxy 
    - Can be installed using these [directions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- CycloneDDS
    - Can be installed by doing the following: ```sudo apt install ros-foxy-rmw-cyclonedds-cpp```
    - This either needs to be sourced each time by running ```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``` or automatically by adding it to the bashrc file.
- Docker Engine (optional - useful if a premade image is desired)
- OpenCV - the latest version is OK
    - Install can be found [here](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
    - Note that Jacob highly recommends going directly to detailed install process. The quickstart makes it easy to make a mistake!
- Python 3.8.10
- Pyserial - latest version is OK. Can be installed with:
python3 -m pip install pyserial
 - STM32CubeMX or STM32CubeIDE
    - CubeMX is fine if you are comfortable with Makefiles. Otherwise, use the IDE which makes flashing the board as easy as pressing a play button 





## Hardware Requirements
- 2 Raspberry Pi 4Bs were used with 8 GB of RAM (one for the submarine, one for the controller). It may be possible to achieve success with less, but bear in mind there is much OpenCV work being done on the controller side to run the display.
- STM32 Nucleo-L4R5ZI (For submarine)
- STM32 Nucleo-L452RET-P (For controller)
## Installation

Since this is a large project, installs can sometimes get pretty messy. For example, ROS2 uses ```colcon build``` to prepare an executable. This is very handy and incredibly powerful. _However_ there isn't any sane way to rein in what colcon attemps to build, and surprise surprise, when colcon expects python and runs across C source code, it has no idea what to do. To alleviate this, we have made subdirectories called ros2_ws (which acts as our root ROS2 folder for all nodes) and stm32_ws (which acts as the root STM workspace for both the controller and submarine).

To actually download/install, follow these steps:

1)
```git clone https://github.com/jacobcwildes/Submarine_Capstone.git```

2) 
```cd Submarine_Capstone/ros2_ws```

3)
```source /opt/ros/foxy/setup.bash```

4) Note: **Before** running this step, make sure you are NOT in the ros2_ws/src directory. When colcon builds it makes 3 directories. An install, a build, and a log directory. When we go to run a ROS2 node, we first source the bash script in the install folder. The way Jacob has set up the launchfiles, the node WILL fail if the folders are not properly built.
```colcon build```

5) Once the build is finished
```source install/setup.bash```

In order to run the STM side (to query controller inputs and also run the submarine motors)
***Add screenshots of the IDE & Makefile gen here**

Thankfully, once a board is flashed it stays flashed. Once the board is powered, it will immediately begin whatever program is on it.
## Docker
If these installation steps sound gross, no sweat! Jacob has made a docker image which is able to run on just about any 64 bit version of Linux. MacOS and Windows may be compatible, but are untested. In order to use the provided images, do the following:

1) Clone this repo
```git clone https://github.com/jacobcwildes/Submarine_Capstone.git```

2) Pull the images down from Docker Hub:
```docker pull jacobcwildes/2023-submarine-capstone:controller-deploy```

```docker pull jacobcwildes/2023-submarine-capstone:submarine-deploy```

These downloads will take a little time, as they are both between 1 and 1.6 GB. 

Alternatively, if you desire to build the Docker images locally, repeat step 1 and pull this repo.

Then do the following:

1) ```cd Submarine_Capstone/ros2_ws```. This is where the Dockerfiles are.

2) Build the Controller Docker image (Note that this will take a long time - it must download everything from the Ubuntu:20.04.5 base)
docker build -t controller_image . < controller_Dockerfile

3) Build the Submarine Docker image:
docker build -t controller_image . < sub_Dockerfile

Once the builds are complete, the launch sequence is the same for both methods of installation.  Simply run

```docker compose -f controller_compose.yaml up``` For the controller
```docker compose -f sub_compose.yaml up``` For the submarine
