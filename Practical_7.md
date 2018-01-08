Robotic Arm Mini-project Part 2 - ROS
=====================================
Step 2 - First steps with ROS
-----------------------------
When using ROS, `roscore` first needs to be launched:

```console
ben@b-gordon:~$ roscore
... logging to /home/ben/.ros/log/d2504b20-dfa8-11e7-bd44-3464a9c0be15/roslaunch-b-gordon-9982.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://b-gordon:32785/
ros_comm version 1.12.12


SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.12

NODES

auto-starting new master
process[master]: started with pid [9993]
ROS_MASTER_URI=http://b-gordon:11311/

setting /run_id to d2504b20-dfa8-11e7-bd44-3464a9c0be15
process[rosout-1]: started with pid [10006]
started core service [/rosout]

```

Once launched, other ROS commands can be used.
First was using `rostopic`

```console
ben@b-gordon:~$ rostopic list
/rosout
/rosout_agg
```

As can be seen this displays two files.
According to the ROS wiki, `/rosout` is used for log reporting. It'll record the messages and store them in a text log files and
rebroadcasts them on `/rosout_agg`.

To publish on a new topic I use the following command:
```console
ben@b-gordon:~$ rostopic pub /test std_msgs/String "Hello"
publishing and latching message. Press ctrl-C to terminate
```

Now when listing `rostopic` 3 items can be seen:
```console
ben@b-gordon:~$ rostopic list
/rosout
/rosout_agg
/test
```
The bottom most one can be seen to be the topic whe just created `/test`.


Upon opening a new terminal, the console messages can be seen from `/test`:
```console
ben@b-gordon:~$ rostopic echo /test
data: "Hello"
---
```
Now when publishing to the topic, will display here. The following shows when changing the text three times:
```console
ben@b-gordon:~$ rostopic echo /test
data: "Hello"
---
data: "This is the 1st"
---
data: "Here's another"
---
data: "One last time!"
---
```

Step 3 - RVis
-------------
RVis can be started by typing `rvis` in the terminal:
```console
ben@b-gordon:~$ rviz
[ INFO] [1513172101.975307629]: rviz version 1.12.13
[ INFO] [1513172101.975403008]: compiled against Qt version 5.5.1
[ INFO] [1513172101.975438626]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1513172102.838869035]: Stereo is NOT SUPPORTED
[ INFO] [1513172102.839542774]: OpenGl version: 3 (GLSL 1.3).

```

Step 4 - Configure ROS for the Arduino
--------------------------------------
For the arduino board, the cable needs to be used as a serial bridge as the board does not have an ethernet port `rosserial` can be used to do this for ROS messages.

First rosserial needs to be installed:
```console
ben@b-gordon:~$ sudo apt install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino
[sudo] password for ben: 
Reading package lists... Done
Building dependency tree       
.
.
.
Need to get 121 kB of archives.
After this operation, 971 kB of additional disk space will be used.
Do you want to continue? [Y/n]  
Get:1 http://packages.ros.org/ros/ubuntu xenial/main amd64 ros-kinetic-rosserial-msgs amd64 0.7.7-0xenial-20171129-142441-0800 [29.1 kB]
.
.
.
Setting up ros-kinetic-rosserial-client (0.7.7-0xenial-20171129-143316-0800) ...
Setting up ros-kinetic-rosserial-arduino (0.7.7-0xenial-20171129-143838-0800) ...
Setting up ros-kinetic-rosserial-python (0.7.7-0xenial-20171129-143249-0800) ...

```
This installs the packages, however it then needs to be made visible within the arduino IDE as an Arduino library:
```console
ben@b-gordon:~$ cd $HOME/sketchbook/libraries
ben@b-gordon:~/sketchbook/libraries$ rosrun rosserial_arduino make_libraries.py .

Exporting to .
Exporting actionlib

  Messages:
.
.
.

  Messages:
    InteractiveMarker,ImageMarker,InteractiveMarkerInit,MenuEntry,InteractiveMarkerControl,InteractiveMarkerPose,InteractiveMarkerFeedback,InteractiveMarkerUpdate,MarkerArray,Marker,

```
Once this is done, the library can then be accessed from the arduino IDE:


Step 5 - Write a ROS node for your Arduino
------------------------------------------
For this I used the following supplied code (With some added notes)
```cpp
//Include Packages
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

using namespace ros; //location

NodeHandle nh; //Node declare
Servo servo; //Servo declare

void cb(const std_msgs::UInt16 & msg) //Uses 16 bit (UInt16) message as input, writes the data of
{                                     //the message to the servo.
  servo.write(msg.data); //Write 0 - 180 to servo 
}

Subscriber<std_msgs::UInt16> sub("servo", cb); //Creates sub of "servo" cb

void setup()
{
  nh.initNode(); //Initialise Node
  nh.subscribe(sub);
  
  servo.attach(9); //Attach to pin 9
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
```
I then compiled this onto the board. This initially doesnt do anything as the ROS is needed to control it. Firstly `rosrun`
needs to be used to connect ROS to the board:
```console
ben@b-gordon:~$ rosrun rosserial_python serial_node.py /dev/ttyACM2
[INFO] [1513175978.110677]: ROS Serial Python Node
[INFO] [1513175978.132362]: Connecting to /dev/ttyACM2 at 57600 baud
[INFO] [1513175980.274828]: Note: subscribe buffer size is 280 bytes
[INFO] [1513175980.276040]: Setup subscriber on servo [std_msgs/UInt16]

```
`rostopic` can then be used to publish the specified value to the board in order to make the motor turn (0-180):
```console
ben@b-gordon:~$ rostopic pub /servo std_msgs/UInt16 "data: 180"
publishing and latching message. Press ctrl-C to terminate
^C  
ben@b-gordon:~$ 
ben@b-gordon:~$ rostopic pub /servo std_msgs/UInt16 "data: 0"
publishing and latching message. Press ctrl-C to terminate

```
Here I made the servo rotate to 180 degrees then to 0 degrees.
Below shows the new topic created for this use:
```console
ben@b-gordon:~$ rostopic list
/diagnostics
/rosout
/rosout_agg
/servo

```

Part II - 3D model of your arm
==============================
Step 1 - Visualise an existing URDF file in RViz
------------------------------------------------

Firstly I created a directory for the robot then a sub-directory for the models. Within this folder I created a new file 
`robot-arm.urdf`:

```console
ben@b-gordon:~/Desktop$ mkdir robot-project
ben@b-gordon:~/Desktop$ cd robot-project
ben@b-gordon:~/Desktop/robot-project$ mkdir models 
ben@b-gordon:~/Desktop/robot-project$ cd models
ben@b-gordon:~/Desktop/robot-project/models$ gedit robot-arm.urdf

```

Within this I provided the following code given for this task:
```xml
<?xml version="1.0"?>
<robot name="roco_arm">
	<link name="base_link">
		<visual>
			<geometry>
				<cylinder length="0.06" radius="0.1"/>
			</geometry>
		</visual>
	</link>
	
	<link name="first_segment">
		<visual>
			<geometry>
				<box size="0.6 0.05 0.1"/>
			</geometry>
			<origin rpy="0 0 0" xyz="-0.3 0 0"/>
		</visual>
	</link>
	
	<joint name="base_to_first" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
		<parent link="base_link"/>
		<child link="first_segment"/>
		<origin xyz="0 0 0.03"/>
	</joint>
</robot>
```

This file then needs to be set as the description of the robot:
```console
ben@b-gordon:~/Desktop/robot-project/models$ rosparam set robot_description -t robot-arm.urdf
```

I then need to run the `robot_state_publisher` and `joint_state_publisher`:
```console
ben@b-gordon:~$ rosrun robot_state_publisher robot_state_publisher 
```
```console
ben@b-gordon:~$ rosrun joint_state_publisher joint_state_publisher _use_gui:=true
[INFO] [1513178238.628769]: Centering
```

This then opens up a gui:
***insert pic***

Then a robot model visualisation needs to be added to RViz:
*** another pic***

Step 2 - Create the URDF file of your robot
-------------------------------------------
The next step is, using the *robot-arm.urdf* as a template, to create a new file in order to
visualise the robot arm we have created within rviz.

The first step was to make a **part** for each of the sections of the arm. In our case there's:
* **2x Claws**
* **1x Claw Arm Segment**
* **2x Standard Arm Segments**
* **1x Upper Base**
* **1x Lower Base**

The next step is creating the joints which will join all the sections together. For this 
6 Joints are required:
* **1 joinging the lower to upper base**
* **1 joining the first arm to the base**
* **2 joining the 3 Arm segments together**
* **2 joining the claws to the claw arm** ***(which will be controlled together by 1 servo)***


Part III - Control the servo-motors from the robot's joint state
================================================================
