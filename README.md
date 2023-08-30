
# 2023 Submarine Capstone
This GitHub repo will go over the entire process of making a submersible ROV using custom PCBs, and almost entirely 3D printed hull pieces. This project was borne from ideas pooled by Dyllon Dunton and Jacob Wildes to fulfill a senior Capstone at the University of Maine. The end goal of this project is to have a submersible drone which can be operated by human input, and all relevent telemetry (i.e. forward velocity, and depth) will be displayed to the user. 


## Software Requirements
- Machine must be running Ubuntu 20.04.5 LTS
- ROS2 Foxy preinstalled
    - Can be installed using these directions: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Docker Engine (optional - useful if a premade image is desired)
- OpenCV - the latest version is OK
    - Install can be found here: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
    - Note that Jacob highly recommends going directly to detailed install process. The quickstart makes it easy to make a mistake!
- Python 3.8.10
- Pyserial - latest version is OK. Can be installed with:
python3 -m pip install pyserial
 -




## Hardware Requirements
- 2 Raspberry Pi 4Bs were used with 8 GB of RAM (one for the submarine, one for the controller). It may be possible to achieve success with less, but bear in mind there is much OpenCV work being done on the controller side to run the display.
- STM32 Nucleo-L4R5ZI (For submarine)
- STM32 Nucleo-L452RET-P (For controller)