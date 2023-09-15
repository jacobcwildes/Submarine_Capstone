
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
- STM32 Nucleo-L452RE-P (For controller)
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

Note: This must be done twice, once for each Pi. Make very sure that the designated IP address for the eth0 port is not the same on both Pis! It absolutely will not work if both Pis have the same. 

## Software Install
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

## Creating a Mountpoint for the USB drive

First, make sure that the USB drive you are trying to flash is in FAT32 type. It can be any size as we are only saving desired screenshots to the drive. Once it is formatted (if necessary at all), plug into the Pi and do the following:

1)
```bash
sudo fdisk -l
```
Find the drive that corresponds to your drive. I found it is easiest to identify by size and type. If you know it is FAT32 and is 16GB in size, you can deduce the device mountpoint. In our case, it was "/dev/sdb1"

2) Next we need to get the UUID of the drive by running the following:
```bash
sudo ls -l /dev/disk/by-uuid/
```
Look for the entry that corresponds to your mountpoint. For example, our UUID was "A097-2639"

3) Now we need to make a folder for the USB to mount to
```bash
sudo mkdir /mnt/usb
```
The folder name can be whatever you want, but for simplicity I went with "usb"

4) Now we're going to modify the fstab file so that the drive will _always_ be mounted to the "/mnt/usb" partition
```bash
sudo nano /etc/fstab
```

5) Add the following line to the end
```bash
UUID=A097-2639 /mnt/usb vfat uid=${USER},gid={USER} 0 0
```
UUID is your drive ID, and ${USER} is your system username. 

6) Reboot or immediately test by running:
```bash
sudo mount -a
```
If all went well, the drive will be mounted to "/mnt/usb"!
## Flashing STM Boards

_Installing with STM32CubeIDE_

First, make sure that the IDE is installed on your computer. It can be installed from [STM's website](https://www.st.com/en/development-tools/stm32cubeide.html).
Once the software is installed, open the IDE and navigate first to the controller_stm.ioc which can be found in ```Submarine_Capstone/stm32_ws/controller_stm```. Once the project is open in the IDE, the next step is to press the green play button on the top of the screen to flash the Nucleo-L452RE-P with the controller program as shown below
![STM32CubeIDE Steps](https://github.com/jacobcwildes/Submarine_Capstone/blob/main/readme_imgs/IDE_Flash.png)

You will know when the program flashes when the LED next to the USB connection illuminates green.

Repeat this step for the submarine by navigating to ```Submarine_Capstone/stm32_ws/submarine_stm/Submarine_MicroController```

_Installing with STM32CubeMX_

Installing with the CubeMX is a little bit different than the IDE - it is significantly more involved, but for some is the preferred method (especially if your device does not support the IDE). 

First, make sure that you have arm-none-eabi installed on your system. This step can be an absolute nuisance, so the steps are listed below:

1) Check to make sure that you don't have any arm libraries lurking that could foul the download:
```bash
sudo apt remove gcc-arm-none-eabi
```
2) Download the [arm-none-eabi-gcc tarball](https://developer.arm.com/downloads/-/gnu-rm). Note this guide is specifically for Linux. MacOS and Windows mileage will vary from this  guide.

3) Unpack the tarball into some sort of directory. In this case we arbitrarily chose /usr/share
```bash
sudo tar xjf gcc-arm-none-eabi-YOUR-VERSION.bz2 -C /usr/share/
```
"YOUR-VERSION" is whatever version ARM happens to have released at the time

4) Create symlinks from the install folder to the /usr/bin folder like so:
```bash
sudo ln -s /usr/share/gcc-arm-none-eabi-YOUR-VERSION/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc 
sudo ln -s /usr/share/gcc-arm-none-eabi-YOUR-VERSION/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -s /usr/share/gcc-arm-none-eabi-YOUR-VERSION/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb
sudo ln -s /usr/share/gcc-arm-none-eabi-YOUR-VERSION/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size
sudo ln -s /usr/share/gcc-arm-none-eabi-YOUR-VERSION/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy
```
Note: These symlinks may or may not be necessary. On some systems Jacob has worked with, the symlinks were a problem. In other systems, they were not. For safety's sake it's easy enough to just create them and prevent headache later.

5) ARM in its infinite wisdom does not give all the necessary dependencies in its README.txt, but these dependencies served well for this project:
```bash
sudo apt install libncurses-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libncurses.so.6 /usr/lib/x86_64-linux-gnu/libncurses.so.5
sudo ln -s /usr/lib/x86_64-linux-gnu/libtinfo.so.6 /usr/lib/x86_64-linux-gnu/libtinfo.so.5
```
Note: The libncurses.so.6/.so.5 may not be the same at a later date.

6) Check to make sure that the arm-none-eabi was properly installed:
```bash
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version
arm-none-eabi-gdb --version
arm-none-eabi-size --version
```
Make should be installed on your system since OpenCV is a prerequisite for this system to work, but just in case it isn't:
```bash
sudo apt update && sudo apt install make
```
Next, navigate to ```Submarine_Capstone/stm32_ws/submarine_stm/Submarine_MicroController```. This is where the Makefile is.
Run
```bash
make -j4
```
Once the build is complete, run the following:
```bash
cp BINARY_PATH /media/${USER}/BOARD_NAME/
```
BINARY_PATH will be filled in soon, BOARD_NAME is the name of the board when it is plugged into the computer.

If all worked well, the board will be successfully flashed!

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
