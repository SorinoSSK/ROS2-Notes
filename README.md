# Robotic Operating System (ROS)
## Introduction
This page serves as my notes while learning ROS2 humble. I will be writing my notes for linux running on windows wsl as I am studying ROS2 from a linux-based system.\
This notes is not intended to take credits from any author. 

**ROS1**\
ROS1 starts with the ROS Master. ROS Master allows all nodes to find and talk to each other. Topic defines the name and types of message that will be sent concerning that topic. The node transmit data by publishing to a topic name and receive data by subscribing to a topic.

**ROS2**\
ROS2 is built on ***Data Distribution Service (DDS)*** which allows better communication than **ROS1**. Improvements such as
- Best-in-class security
- Embedded and real-time support
- Multi-robot communication
- Operations in non-ideal networking environments
## Table of Content
|S/N|Content                    |
|--|---------------------------|
|01|[**Introduction**](#introduction)
|02|[**Enabling wsl**](#enabling-wsl)|
|03|[**Installing Ubuntu 22.04.2**](#installing-ubuntu-22042)|
|04|[**Installing X server**](#installing-x-server)|
|05|[**Creating ROS Package**](#creating-ros-package)|
|05.1|[Modify ROS Package Information](#modifying-package-information)|
|06|[**Building ROS package**](#building-ros-package)|
|06.1|[Updating Linux Environment](#updating-environment)|
|06.2|[Running a node](#running-your-node)|
|07|[**ROS Commands**](#ros-commands)|
|07.1|[List Node](#list-node)|
|07.2|[Get Node Information](#get-node-information)|
|07.3|[List Topic](#list-topic)|
|07.4|[List Topic with Verbose](#list-topic-with-verbose)|
|07.5|[Print all messages published to a topic](#print-all-messages-published-to-a-topic)|
|07.6|[Get Topic Type](#get-topic-type)|
|07.7|[Publish To A Topic Using Command](#publishing-to-a-topic-using-command)|
|07.8|[Getting Publish Rate Of a Topic](#getting-publish-rate-of-a-topic)|
|07.9|[Remapping of Node](#remapping-of-node)|
|08|[**RQT**](#rqt)
|08.1|[Installing RQT](#installing-rqt)|
|08.2|[Running RQT](#running-rqt)|
|08.3|[RQT Graphing](#rqt-graphing)|
|08.4|[RQT Plot](#rqt-plot)|
|09|[**ROS Service**](#ros-service)|
|09|[**ROS Param**](#ros-service)|
|10|[**Turtle Sims**](#turtle-sims)|




## Enabling wsl
**Step 1:** Search for **Windows Features**. 

**Step 2:** Enable **Virtual Machine Platform** and **Windows Subsystem for Linux**.

![Alt text](/Asset_Image/Enable%20VM.png "Enable VM for windows")

## Installing Ubuntu 22.04.2
**Step 1:** Go to Windows Store and search for **Ubuntu 22.04.2 LTS**.

**Step 2:** Download **Ubuntu 22.04.2 LTS** from the Windows Store.

**Step 3:** Open the Ubuntu terminal by clicking on the icon of your newly installed application.

![Alt text](/Asset_Image/Ubuntu%20Icon.png "Ubuntu Icon")

**Step 4:** Register an ubuntu account with the initial prompt from your ubuntu terminal.

## Installing X server
Since we are running ubuntu using wsl, we will need to setup an application for Linux GUI application to load. X server is a suggested third-party application for us to run Linux GUI application on wsl. You will need to run the application everytime you restart your computer. 

You could also read more about the application [here](https://aalonso.dev/blog/how-to-use-gui-apps-in-wsl2-forwarding-x-server-cdj) and install the application [here](https://sourceforge.net/projects/vcxsrv/).

In addition, follow the following steps to setup your linux system to send the display to X server.  You will need to run these steps for every new terminal.

**Step 1:** Open an empty file.
```
nano monitorDisplay
```

**Step 2:** Paste the following instructions into the file.
```
#!/bin/sh

export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
export DISPLAY="`sed -n 's/nameserver //p' /etc/resolv.conf`:0"
export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0
export DISPLAY=192.168.150.1:0.0
```
*Note: You can find your ip address using* ***ifconfig*** 

**Step 3:** Set your ***linux environment*** so ***all display will be pushed to X Server***.
```
source monitorDisplay
```

## ROS basic terminology
1) **Nodes**\
A node is an executable that uses ROS to communicate with other nodes.

2) **Messages**\
ROS data type used when subscribing or publishing to a topic.

3) **Topics**\
Nodes can publish to a topic as well as subscribe to a topic to receive messages.

4) **Master**\
Name service for ROS *(For nodes to find each other)*

5) **rosout**\
ROS equivalend of stdout/stderr

6) **roscore**\
Master + rousout + parameter server

## Creating ROS Package
You could create a ROS package with the following command;
```
ros2 pkg create --build-type ament_python --node-name <node_name> <package_name> 
```
Where ***<node_name>*** and ***<package_name>*** could be any name of your preference.

To create an **empty ROS package**, you could use the following command;
```
ros2 pkg create --build-type ament_python <package_name> 
```
### Modifying package information
You could modify the ROS package information in the ***package.xml*** file inside
```
<You Directory Root>/src/<package_name>
```
An example of the package information:
```
<description>This is the package description for Task 2</description>    <maintainer email="jsmith@todo.todo">johnsmith</maintainer>  <license>Apache License 2.0</license> 
```
## Building ROS Package
To build a ROS package, you will be required to install rosdep. You could ***install rosdep with the following command*** or ***skip this step and follow your terminal instructions*** when building.
```
sudo apt install python3-rosdep2
```

Follow the following steps to build a ROS package.
**Step 1:**
```
sudo rosdep init
```
```
rosdep update
```
**Step 2:**
```
colcon build
```
However, you could also ***modify a node without having to rebuild*** the entire package with;
```
colcon build --symlink-install
```
You could too build only your package file.
```
colcon build --packages-select <package_name>
```
### Updating environment
After every build, you should ***update your environment*** with your ***new package***. 
```
source install/local_setup.bash 
```
### Running your node
After building your package and setting up your environment, you could then proceed to run your node.
```
ros2 run <package_name> <node_name>
```
## ROS commands
### List node
```
ros2 node list
```
### Get node Information
```
ros2 node info /<node_name>
```
### List Topic
```
ros2 topic list
```
### List Topic with Verbose
```
ros2 topic list -v
```
### Print all messages published to a topic
```
ros2 topic echo <topic_name>
```
### Get topic type
```
ros2 topic type <topic_name>
```
### Publishing to a topic using command
```
ros2 topic pub <method> <topic_name> <msg_type> '<args>'
```
e.g.
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
```
```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
```
where ***geometry_msgs/msg/Twist*** contain message type of
```
Vector3 linear
    float64 x
    float64 y
    float64 z
Vector3 angular
    float64 x
    float64 y
    float64 z 
```
### Getting publish rate of a topic
```
ros2 topic hz <topic_name>
```
e.g.
```
ros2 topic hz /turtle1/cmd_vel
```
### Remapping of node
Remapping allow us to map the properties of a node to another node. For instance, if we could get a node to subscribe to the same topic of another node.
```
ros2 run <package_name> <node_name> --ros-args --remap __node:=<node_name_mapped_to>
```
e.g.
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle 
```
However, if you would like to map the properties only, you should differentiate the topic.
```
ros2 run <package_name> <node_name> --ros-args --remap __node:=<node_name_mapped_to>  –-remap <topic_name>:=<topic_name_of_node_mapped_to> 
```
e.g.
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle  –-remap turtle1/cmd_vel:=my_turtle/cmd_vel 
```
### Overlays
Overlays allows use to modify the properties of a Linux UI. For instance, we could change the window title and background of the turtle sim UI.

There are two methods for us to modify the UI.

1) **Modify in the file manage**
```
<You Directory Root>/src/<package_name>/<package_name>/src
```
e.g.
```
ros2_ws/src/ros_tutorials/turtlesim/src
```
and open the file called ```turtle_frame.cpp```

2) **Using command prompt**

e.g.
```
setWindowTitle("TestSample"); 
```
After modifying the the UI properties, we could rebuild the package. You do not have to close the terminal or window for ***method 2*** of the above.

```
colcon build
```
```
ros2 run <package_name> <node_name>
```
or
```
ros2 run turtlesim turtlesim_node
```
### List all services
Print information about active services.
```
ros2 service list
```
### Call a service
Call a service with provided arguments.
```
ros2 service call <service_name> <service_type> <arguments>
```
e.g.
```
ros2 service call /clear std_srvs/srv/Empty
```
### Get a service type
Prints service type.
```
ros2 service type <service_name>
```
### Find a service
Print services by service type.
```
ros2 service find <service_type>
```
### Get service's input argument
***<service_type>*** can be retrieved from "[**Get a service type**](#get-a-service-type)".
```
ros2 interface show <service_type>
```
### List all param
This will list out all available node's parameters.
```
ros2 param list
```
e.g. Output
```
/teleop_turtle:
    scale_angular
    scale_linear
    use_sim_time
/turtlesim:
    background_b
    background_g
    background_r
    use_sim_time
```
### Set a param
This function is similar to a setter function.
```
ros2 param set <node_name> <parameter_name> <value>
```
e.g.
```
ros2 param set /turtlesim background_r 150
```
### Get a param
This function is similar to a getter function.
```
ros2 param get <node_name> <parameter_name>
```
e.g.
```
ros2 param get /turtlesim background_g
```
### Get all param values (param dump)
This function get all param value of a node.
```
ros2 param dump <node_name>
```
You could store param values by echoing them into a file.
```
ros2 param dump /turtlesim > myparams.yaml
```
### Loading param from yaml file
This function loads a yaml file parameter values to a node.
```
ros2 param load <node_name> <directory/parameter_file>
```
e.g.
```
ros2 param load /turtlesim myparams.yaml
```
### Executing a node with a param file
You could run a node and initialise it with your param file.
```
ros2 run <package_name> <node_name> --ros-args --params-file <file_name>
```
e.g.
```
ros2 run turtlesim turtlesim_node --ros-args --params-file myparams.yaml
```
## RQT
RQT is a graphical user interface that gave an UI interaction with ROS.

![Alt text](/Asset_Image/RQT%20UI.png "RQT User Interface")

### Installing RQT
```
sudo apt install ~nros-humble-rqt*
```
### Running rqt
```
rqt
```
### RQT graphing
RQT graph is an interface that allow us to visualise how all available nodes and topics communicate with each other.
```
ros2 run rqt_graph rqt_graph
```
![Alt text](/Asset_Image/RQT%20Graph.png "RQT Graphs")

### RQT Plot
RQT plot plots data that are being published to a topic.
```
ros2 run rqt_plot rqt_plot
```
![Alt text](/Asset_Image/RQT%20Plot.png "RQT Plot")

## ROS Service
ROS service is an alternative for nodes to communicate by sending a request and receiving a response. It performs similarly to sending a publishing to a topic using :
```
ros2 topic pub <method> <topic_name> <msg_type> '<args>'
```

ROS service has the following functions:
|                 |                                        |
|-----------------|----------------------------------------|
|[ros2 service list](#list-all-services)| print information about active services|
|[ros2 service call](#call-a-service)| call the service with the provided args|
|[ros2 service type](#get-a-service-type)| print service type                     |
|[ros2 service find](#find-a-service)| find services by service type          |
## ROS Param
ROS param allows us to store and manipulate data on the ROS Paramerter Server. ROS Parameter server uses a YAML markup language to store information such as integers, float, boolean, and list.

ROS param has the following functions:
|                   |                                       |
|-------------------|---------------------------------------|
|[ros2 param set](#set-a-param)|set parameter|
|[ros2 param get](#get-a-param)|get parameter|
|[ros2 param load](#loading-param-from-yaml-file)|load parameters from file|
|[ros2 param dump](#get-all-param-values-param-dump)|dump parameters to file|
|ros2 param delete  |delete parameter|
|[ros2 param list](#list-all-param)|list parameter names|
|ros2 param describe| Show descriptive information about parameters|


## Turtle Sims
Turtle Sims is a program that allows user to better learn about the concepts of Nodes and Topics.

### Installing Turtle Sims
**Step 1:**
```
sudo apt install ros-humble-turtlesim
```
**Step 2:** Verify if turtle sim is installed.
```
ros2 pkg executables turtlesim
```
### Running Turtle Sims
To run turtle sims use the following command;
```
ros2 run turtlesim turtlesim_node
```
The above will generate a Linux GUI application that open up a node and subscribe it to a topic.

*Note: Since we are using wsl, the turtle sims will fail to launch thus you should enable the display by setting your environment for* ***[X server](#installing-x-server)***.

To move the turtle in the turtle sim, we could launch a node to send messages to the topic.
```
ros2 run turtlesim turtle_teleop_key
```
You could run the turtle sim using commands only such as;
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
```
```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
```

### List a node information
To list information of a node, we could use the following command:
```
ros2 node info /<node_name>
```
For example, we could get information of the ***teleop_turtle*** with;
```
ros2 node info /teleop_turtle
```