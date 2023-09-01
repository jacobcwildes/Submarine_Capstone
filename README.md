
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

## Pi Setup

As per usual, it is never a super easy process to set anything up. To flash the Raspberry Pi Jacob opted to use the official [Raspberry Pi Imager](https://www.raspberrypi.com/software/). This tool allows for easy flashing of SD cards. However, when perusing the options to install the Ubuntu environment, as of the date of flashing (7/23), there was no 64-bit Ubuntu 20.04.5 LTS desktop option. The only option was a server version. Without much other option, the server option works as a good enough base. In Jacob's experience, even if a Wi-Fi setting is given in the imager, it still fails to find a network connection when the server is booted (note that the server version is only a command line). 

In order to solve these networking woes, we need to modify our netplan file. This can be done by running 
```sudoedit /etc/netplan/<filename>.yaml``` 
where "filename" is the first entry that shows up when you hit tab a couple times. In Jacob's case it was 50-cloud-init.yaml, though your mileage may vary. 
Once inside the document, you will want to paste the following into it:
```bash

network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
            addresses:
                - <your_desired_address>/24
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                <Wi-Fi_name>:
                    password: <your_password>
            dhcp4: true
            optional: true

```
"your_desired_address> should be something like 192.168.1.X, but it can be whatever you like. This makes it so when we attach both Pis together, they have a static IP and can immediately talk to each other. "Wi-Fi_name" is the name of your wireless network. "your_password" is the password of your wireless network. 

The eth0 option is needed because by default the Pi will not pick up the ethernet connection. This is probably because the gnome desktop is installed over top of the server installation, and the GUI functionality does not work.

Once that is done, it is time to apply those changes. Run the following:
```bash
sudo netplan generate && sudo netplan apply
```
This command will take some time, but afterward the Pi should be attached to the web!

## Getting the Ubuntu Desktop
Now that the Pi is on the web, it is important to actually get our GUI installed. To do so run the following:
```bash
sudo apt update && sudo apt upgrade -y
```
```bash
sudo apt install sudo apt install ubuntu-desktop -y && sudo reboot
```
Once the desktop is installed, you will be ready to download/run the nodes!


1)
```bash
git clone https://github.com/jacobcwildes/Submarine_Capstone.git
```

3)
```bash
cd Submarine_Capstone/ros2_ws
```

4)
```bash
source /opt/ros/foxy/setup.bash
```

6) Note: **Before** running this step, make sure you are NOT in the ros2_ws/src directory. When colcon builds it makes 3 directories. An install, a build, and a log directory. When we go to run a ROS2 node, we first source the bash script in the install folder. The way Jacob has set up the launchfiles, the node WILL fail if the folders are not properly built.
```bash
colcon build
```

8) Once the build is finished
```bash
source install/setup.bash
```

In order to run the STM side (to query controller inputs and also run the submarine motors)
***Add screenshots of the IDE & Makefile gen here**

Thankfully, once a board is flashed it stays flashed. Once the board is powered, it will immediately begin whatever program is on it.

## Docker
If these installation steps sound gross, no sweat! Jacob has made a docker image which is able to run on just about any 64 bit version of Linux. MacOS and Windows may be compatible, but are untested. In order to use the provided images, do the following:

1) Clone this repo
```bash
git clone https://github.com/jacobcwildes/Submarine_Capstone.git
```

3) Pull the images down from Docker Hub:
```bash
docker pull jacobcwildes/2023-submarine-capstone:controller-deploy
```

```bash
docker pull jacobcwildes/2023-submarine-capstone:submarine-deploy
```

These downloads will take a little time, as they are both between 1 and 1.6 GB. 

Alternatively, if you desire to build the Docker images locally, repeat step 1 and pull this repo.

Then do the following:

1) This is where the Dockerfiles are.
```bash
 cd Submarine_Capstone/ros2_ws
```

2)
  Build the Controller Docker image (Note that this will take a long time - it must download everything from the Ubuntu:20.04.5 base)
```bash
docker build -t controller_image . < controller_Dockerfile
```

4) Build the Submarine Docker image:
```bash
docker build -t controller_image . < sub_Dockerfile
```

Once the builds are complete, the launch sequence is the same for both methods of installation.  Simply run:

For the controller:
```bash
docker compose -f controller_compose.yaml up
```
For the submarine:
```bash
docker compose -f sub_compose.yaml up
``` 
