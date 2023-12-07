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
|07.10|[Overlays](#overlays)|
|07.11|[List all Services](#list-all-services)|
|07.12|[Call a Service](#call-a-service)|
|07.13|[Get a Service Type](#get-a-service-type)|
|07.14|[Find a Service](#find-a-service)|
|07.15|[Get Service's Input Argument](#get-services-input-argument)|
|07.16|[List all Params](#list-all-param)|
|07.17|[Set a Param](#set-a-param)|
|07.18|[Get a Param](#get-a-param)|
|07.19|[Get all Param Values](#get-all-param-values-param-dump)|
|07.20|[Loading Param From YAML File](#loading-param-from-yaml-file)|
|07.21|[Executing node with Param File](#executing-a-node-with-a-param-file)|
|08|[**RQT**](#rqt)
|08.1|[Installing RQT](#installing-rqt)|
|08.2|[Running RQT](#running-rqt)|
|08.3|[RQT Graphing](#rqt-graphing)|
|08.4|[RQT Plot](#rqt-plot)|
|09|[**Launch File**](#launch-file)|
|11.1|[Executing Launch File](#executing-a-launch-file)|
|11.2|[Creating Launch File](#creating-a-launch-file)|
|10|[**ROS Param**](#ros-service)|
|11|[**ROS Service**](#ros-service)|
|11.1|[Creating ROS Service](#creating-a-custom-ros-service)|
|12|[**Custom ROS Message**](#ros-message)|
|12.1|[Creating ROS Message](#creating-a-custom-ros-message)|
|12.2|[Creating Publisher Node](#create-a-publisher-node-for-your-message)|
|12.3|[Creating Subscriber Node](#create-a-subscriber-node)|
|13|[**Turtle Sims**](#turtle-sims)|

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
If you would like to build multiple packages:
```
colcon build --packages-select <package_name1> <package_name2>
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
***Notes: Some dependencies may be directory sensitive.***\
***E.g. Dependencies that tries to retrieve OS current directory will be affected by where you execute ros2 run***
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
Print information of active services.
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

## Launch File
A launch file allow ROS to launch application using scripts or to execute a list of ROS function.

### Executing a launch file
```
ros2 launch <package> <filename.launch.py>
```
e.g.
```
ros2 launch turtlesim multisim.launch.py
```

### Creating a launch file
For convention, your launch file will reside within your package folder. Therefore to create a new launch file, you could do the following:

**Step 1:** Go to your package directory.
```
cd ~/<Your Directory>/src/<package_name>
```
e.g.
```
cd ~/ros2_ws/src/my_package
```
**Step 2:** Create a launch folder and enter the directory
```
mkdir launch
```
```
cd launch
```
**Step 3:** Create your launch file
```
nano <file_name>.launch.py
```
e.g.
```
nano my_file.launch.py
```
**Step 4:** Program your launch file\
You could reference the following to generate your launch file.
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )
])
```
Alternatively you could access ROS's turtle sim launch file to reference. The directory of the turtle sim launch file will be here; ```/opt/ros/humble/share/turtlesim/launch/multisim.launch.py```

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

## ROS Service
ROS service is an alternative for nodes to communicate by sending a request and receiving a response. ***ROS Service allows user to send arguments through through "ros2 run"***. It performs similarly to sending a publishing to a topic using :
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

### Creating a custom ROS Service
**Step 1:** Go to your ***src*** directory.
```
cd ~/<your_directory>/src
```
**Step 2:** Create an interface directory.
```
ros2 pkg create --build-type ament_cmake <interface_name>
```
**Step 3:** Access the newly generated interface directory.
```
cd <interface_name>
```
**Step 4:** Create a directory for your service files.
```
mkdir <service_folder_name>
```
**Step 5:** Create a service file.
```
nano service_file_name.srv
```
e.g.
```
nano MyService.srv
```
**Step 6:** Define your service's input type in your service file.
```
int64 val1
float64 val2
```
**Step 7:** Declare your service file for building inside package.xml file stored in your <interface_name> directory.
```
nano package.xml
```
Type the following into the end of ***\<package> \</package>*** element of your package.xml file.\
You need not enter ```<depend><dependent_service></depend>``` if your program is not dependent on a service type.
```
<depend><dependent_service></depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
e.g.
```
<package format="3">
    .
    .
    .
    <export>
        <build_type>ament_cmake</build_type>
    </export>

    <depend><dependent_service></depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```
**Step 8:** Define your service file location for your service file declared in ***Step 7***.\
Open ***CMakeLists.txt*** within your ***\<interface>*** folder.
```
nano CMakeLists.txt
```
Insert the following line into your ***CMakeLists.txt*** file.\
You too need not require to enter ```DEPENDENCIES <service_folder_name>``` if your service is not dependent on other service type.
```
# find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "<service_folder_name>/<service_file_name1>.srv"
    "<service_folder_name>/<service_file_name2>.srv"
    DEPENDENCIES <service_folder_name> # Add packages that above service depend on
)
```
**Step 9:** Go to the root directory of your ROS package. (Directory that contain ***/src***).
```
cd ~/<your_directory>
```
**Step 10:** Build the modified ***\<interface_name>***.
```
colcon build --packages-select <interface_name>
```
**Step 11:** Re-source your setup file.
```
source install/setup.bash
```

## ROS Message
ROS message are messages that are being sent to a topic and the argument of these messages are stored as a text file to describe the fields of a ROS message. ***A message is used to send data between nodes programically*** All message text file will reside within the ***\<interface>*** file.

ROS can contain the Following field types.
- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C] 

### Creating a custom ROS message
**Step 1:** Go to your ***src*** directory.
```
cd ~/<your_directory>/src
```
**Step 2:** Create an interface directory.
```
ros2 pkg create --build-type ament_cmake <interface_name>
```
**Step 3:** Access the newly generated interface directory.
```
cd <interface_name>
```
**Step 4:** Create a directory for your message files.
```
mkdir <message_folder_name>
```
**Step 5:** Create a message file.
```
nano message_file_name.msg
```
e.g.
```
nano MyMessage.msg
```
**Step 6:** Define your message's input and output type in your message file.
```
int64 val1      // Input message
float64 val2
---
string val3      // Output message
unit8 val4
```
**Step 7:** Declare your message file for building inside package.xml file stored in your <interface_name> directory.
```
nano package.xml
```
Type the following into the end of ***\<package> \</package>*** element of your package.xml file.\
You need not enter ```<depend><dependent_message></depend>``` if your program is not dependent on a message type.
```
<depend><dependent_message></depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
e.g.
```
<package format="3">
    .
    .
    .
    <export>
        <build_type>ament_cmake</build_type>
    </export>

    <depend><dependent_message></depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```
**Step 8:** Define your message file location for your message file declared in ***Step 7***.\
Open ***CMakeLists.txt*** within your ***\<interface>*** folder.
```
nano CMakeLists.txt
```
Insert the following line into your ***CMakeLists.txt*** file.\
You too need not require to enter ```DEPENDENCIES <message_folder_name>``` if your message is not dependent on other message type.
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "<message_folder_name>/<message_file_name1>.msg"
    "<message_folder_name>/<message_file_name2>.msg"
    DEPENDENCIES <message_folder_name> # Add packages that above messages depend on
)
```
**Step 9:** Go to the root directory of your ROS package. (Directory that contain ***/src***).
```
cd ~/<your_directory>
```
**Step 10:** Build the modified ***\<interface_name>***.
```
colcon build --packages-select <interface_name>
```
**Step 11:** Re-source your setup file.
```
source install/setup.bash
```
**Step 12:** Verify your message.
```
ros2 interface show <package_name>/<message_folder_name>/<message_file_name>
```
### Create a publisher node for your message (Create Topic)
**Step 1:** Go to your package folder.
```
cd ~/<your_directory>/src/<package_name>/<package_name>
```
**Step 2:** Create a publisher file.
```
nano <publisher_file_name>.py
```
e.g.
```
publisher_member_function.py
```
**Step 3:** Generate your publisher file.\
You may use the following as a reference.
```
import rclpy
from rclpy.node import Node # To use the Node class
from std_msgs.msg import String # imports the built-in string message type

class MinimalPublisher(Node): # creates a class that inherits from Node
    def __init__(self):
        super().__init__('minimal_publisher') # defines the node name
        message_name = String
        topic_name = 'topic'
        queue_size = 10
        self.publisher_ = self.create_publisher(message_name, topic_name, queue_size)
        
        # Example for publishing every 0.5 second
        timer_period = 0.5 # seconds # executes every 500ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # a counter used in the callback function below

    def timer_callback(self): # creates a message with the counter value appended
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) # published to console
        self.i += 1

def main(args=None): # main function
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher) # loops until destroyed
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
main()
```
**Step 4:** Add dependencies to your ***package.xml*** file in your ***\<package_name>*** folder.
```
nano package.xml
```
You need not require to enter ```<exec_depend><message_file_name></exec_depend>``` if you message file is not dependent on other message files.
```
<exec_depend>rclpy</exec_depend>
<exec_depend><message_file_name></exec_depend>
```
e.g. The following add string as a message argument.
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
**Step 5:** Add dependencies to your ***setup.py*** file in the same directory as "***Step 4***".
```
entry_points={
    'console_scripts': [
        'my_node = <package_name>.<node_name>:main',
        'talker = <package_name>.<publisher_file_name>:main',       // Add this line
    ],
},
```
**Step 6:** Build your package.
```
cd ~/<your_directory>
```
```
colcon build --packages-select <package_name>
```
e.g.
```
colcon build --packages-select my_package
```
**Step 7:** Re-source your ***setup*** file.
```
source install/setup.bash
```
**Step 8:** Run your publisher.
```
ros2 run <package_name> talker
```
### Create a subscriber node
**Step 1:** Go to your package folder.
```
cd ~/<your_directory>/src/<package_name>/<package_name>
```
**Step 2:** Create a subscriber file.
```
nano <subscriber_file_name>.py
```
e.g.
```
subscriber_member_function.py
```
**Step 3:** Generate your subscriber file.\
You may use the following as a reference.
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# nearly identical to the publisher

class MinimalSubscriber(Node):
    # Uses the same topic: ‘topic’
    def __init__(self):
        super().__init__('minimal_subscriber')
        message_name = String
        topic_name = 'topic'
        queue_size = 10
        self.subscription = self.create_subscription(message_name, topic_name, self.listener_callback, queue_size)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        # prints an info message to the cosole along with the data received
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Step 4:** Add dependencies to your ***package.xml*** file in your ***\<package_name>*** folder.

Since the subscriber node uses the same dependencies or the same message type as the publisher, we do not need to add anything new to the ***package.xml*** file.

**Step 5:** Add dependencies to your ***setup.py*** file in the same directory as "***Step 4***".
```
entry_points={
    'console_scripts': [
        'my_node = <package_name>.<node_name>:main',
        'talker = <package_name>.<publisher_file_name>:main',
        'listener = package_name>.<subscriber_file_name>:main',       // Add this line
    ],
},
```
**Step 6:** Build your package.
```
cd ~/<your_directory>
```
```
colcon build --packages-select <package_name>
```
e.g.
```
colcon build --packages-select my_package
```
**Step 7:** Re-source your ***setup*** file.
```
source install/setup.bash
```
**Step 8:** Run your publisher.
```
ros2 run <package_name> listener
```

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