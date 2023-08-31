
# ROS2 Decisionmaking
There are multiple ways to have made this project.  A perfectly viable way would have been back and forth serial transmission of commands from controller to submarine and then a faster ethernet wire for image transfer. However, that is not particularly sustainable or easily scalable. If I wanted to have multiple separate modules receive data, I would have to have some sort of way to pass that same data to multiple recipients. That would require more hardware or a more complex software side to make sure this is accomplished well. 

A prime example of this issue is my (Jacob's) extension into the thesis. I want another (much more powerful computer than Raspberry Pi) to also receive image data from the camera and then pass that along to the controller to display. If I were not to use ROS2, this process would be much less straightforward. I would have to pipe data from the camera to the external computer for interpretation and then to the controller. Then what happens if something goes wrong with the external computer? The camera feed would die and then the sub would be at some unknown point because the operator has just lost visual with it. 

This is the beauty of ROS2. Instead of being direct connections to everything (i.e. Bill talks to Bob and **only** to Bob), Bill is on a loudspeaker and anyone who wants to listen (in ROS2 terminology, subscribe) can. It does not impact publication speed, the only difference is that multiple things can subscribe and receive data all at once. Now Bill can talk to Bob, Frank, and Mary all at once and there isn't the software/hardware equivalent of a tin can on a string so that everyone gets the information.

Before there are woes and worries about specific TCP/UDP settings, ROS2 offers something called Quality of Service (QoS) settings which allows the programmer to explicitly tell ROS that if a message is published and someone is subscribed, they absolutely have to receive and acknowledge that message before anything else is done. ROS2 can be as "reliable" as TCP or as "best-effort" as UDP. The only requirement is that both subscriber and publisher have this table of settings: 

| Publisher    | Subscriber  | Compatible |
|--------------|-------------|------------|
| Best effort  | Best effort | Yes        |
| Best effort  | Reliable    | No         |
| Reliable     | Best effort | Yes        |
| Reliable     | Reliable    | Yes        |

The only scenario where the publisher and subscriber types are incompatible are when the subscriber is expecting a reliable (TCP) connection, but instead receives a best effort (UDP) connection. Unlike TCP, UDP does not acknowledge a verification packet to have a sort of "handshake" agreement. Without that handshake, the subscriber has no idea that it is supposed to be receiving, and therefore will not connect. The same is true here in the ROS2 QoS ecosystem. According to the documentation, if the publisher is configured to be reliable, it may attempt multiple times to transmit each message being published to the reliable subscriber. That of course, depends on another configuration, like history, or keep all options. That allows the publisher to cache all unreceived messages to continuously try and hail the subscriber.

The official documentation on ROS2 QoS can be found [here](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

In addition to this rather compelling reason, ROS2 is the de facto foundation for robotics. When I began my first internship at UMaine, one of the first things that I was introduced to was ROS2. It allows for fast scalability and it is generally stable. If that isn't extra reason enough, during my time at the ASCC, all of the projects I have been apart of have been built on the back of ROS2 Foxy.

There are two final questions to answer. Why in particular ROS2 Foxy? Why ROS2 over ROS1?

There are multipe releases of ROS2. Each adds some functionality or other, but they are undeveloped at the inital release. There are multiple bugs that aren't found immediately, and some are on more recent releases of Ubuntu. Although Ubuntu 20.04.5 LTS is reaching the end of its lifespan (as of August of 2023), it has been around long enough that multiple security patches have been released, and there are a whole host of development libraries that have been released for it already - some that may or may not be available to newer versions. ROS2 Foxy has also reached the end of its lifespan, but since it has been around for a long time, it has sort of become the Windows 7 of ROS2. It is wildly popular among development groups because there have been so many patches released and packages made specifically for Foxy that it is hard to move on to a newer version where those packages may break.

To answer the final question, ROS1 **only** supports TCP connections. That would put an additional strain on the system to try to receive every single frame from the camera. If a couple of frames are dropped here or there, not a big deal. It would be more taxing to constantly grab every single frame than it would be to take whatever can be grabbed by the program.




## Node Structure
![ROS2 Node Structure](https://github.com/jacobcwildes/Submarine_Capstone/blob/main/readme_imgs/ROS2_Node_List.png)