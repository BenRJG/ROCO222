Robotic Arm Mini-project Part 2 - ROS
=====================================
Step 2 - First steps with ROS
-----------------------------
When using ROS, `roscore` first needs to be launched:

<details><summary>

```console
ben@b-gordon:~$ roscore
```

</summary><p>

```console
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

</p></details>

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

<details><summary>

```console
ben@b-gordon:~$ rviz
```

</summary><p>
	
```console
[ INFO] [1513172101.975307629]: rviz version 1.12.13
[ INFO] [1513172101.975403008]: compiled against Qt version 5.5.1
[ INFO] [1513172101.975438626]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1513172102.838869035]: Stereo is NOT SUPPORTED
[ INFO] [1513172102.839542774]: OpenGl version: 3 (GLSL 1.3).
```

</p></details>

Step 4 - Configure ROS for the Arduino
--------------------------------------
For the arduino board, the cable needs to be used as a serial bridge as the board does not have an ethernet port `rosserial` can be used to do this for ROS messages.

First rosserial needs to be installed:


<details><summary>

```console
ben@b-gordon:~$ sudo apt install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino
```

</summary><p>

```console
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

</p></details>

This installs the packages, however it then needs to be made visible within the arduino IDE as an Arduino library:

<details><summary>

```console
ben@b-gordon:~$ cd $HOME/sketchbook/libraries
ben@b-gordon:~/sketchbook/libraries$ rosrun rosserial_arduino make_libraries.py .
```

</summary><p>

```console
Exporting to .
Exporting actionlib

  Messages:
.
.
.

  Messages:
    InteractiveMarker,ImageMarker,InteractiveMarkerInit,MenuEntry,InteractiveMarkerControl,InteractiveMarkerPose,InteractiveMarkerFeedback,InteractiveMarkerUpdate,MarkerArray,Marker,

```

</p></details>

Once this is done, the library can then be accessed from the arduino IDE:


Step 5 - Write a ROS node for your Arduino
------------------------------------------
For this I used the following supplied code (With some added notes)

<details><summary>**ROS Node Code**</summary><p>

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

</p></details>

I then compiled this onto the board. This initially doesnt do anything as the ROS is needed to control it. Firstly `rosrun`
needs to be used to connect ROS to the board:

<details><summary>
	
```console
ben@b-gordon:~$ rosrun rosserial_python serial_node.py /dev/ttyACM2
```

</summary><p>
	
```console
[INFO] [1513175978.110677]: ROS Serial Python Node
[INFO] [1513175978.132362]: Connecting to /dev/ttyACM2 at 57600 baud
[INFO] [1513175980.274828]: Note: subscribe buffer size is 280 bytes
[INFO] [1513175980.276040]: Setup subscriber on servo [std_msgs/UInt16]

```

</p></details>

`rostopic` can then be used to publish the specified value to the board in order to make the motor turn (0-180):
```console
ben@b-gordon:~$ rostopic pub /servo std_msgs/UInt16 "data: 0"
publishing and latching message. Press ctrl-C to terminate
^C  
ben@b-gordon:~$ 
ben@b-gordon:~$ rostopic pub /servo std_msgs/UInt16 "data: 1800"
publishing and latching message. Press ctrl-C to terminate

```

[![PartIII video](https://i.ytimg.com/vi/Ndkt1RIQ37k/sddefault.jpg)](https://youtu.be/Ndkt1RIQ37k)

Here I made the servo rotate to 0 degrees then to 180 degrees.
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

<details><summary>**robot-arm.urdf**</summary><p>
	
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

</p></details>

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
***another pic***

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

Below is the code used for this:

<details><summary> **robot-arm2.urdf** </summary><p>

```xml
<?xml version="1.0"?>
<robot name="roco_arm"> <!--name of robot-->
	
	<!--================Robot Arm Parts=================-->	
	
	<!--<link name="bottom_base"> <!-name of part->
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/base.stl"/> <!-link to model file->
			</geometry>
		</visual>
	</link>
	
	<link name="top_base">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/base1.stl"/>
			</geometry>
		</visual>
	</link>-->
	
	<link name="first_segment">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/arm1.stl"/>
			</geometry>
			<!--origin rpy="0 0 0" xyz="-0.3 0 0"/-->
		</visual>
	</link>

	<link name="second_segment">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/arm1.stl"/>
			</geometry>
			<!--origin rpy="0 0 0" xyz="-0.3 0 0"/-->
		</visual>
	</link>

	<link name="claw_segment">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/claw1.stl"/>
			</geometry>
			
		</visual>
	</link>

	<!--<link name="first_claw">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/clawarm2.stl"/>
			</geometry>
			<!-origin rpy="0 0 0" xyz="-0.3 0 0"/->
		</visual>
	</link>

	<link name="second_claw">
		<visual>
			<geometry>
				<mesh filename = "file:///home/ben/Desktop/robot-project/models/parts/clawarm1.stl"/>
			</geometry>
			<!-origin rpy="0 0 0" xyz="-0.3 0 0"/->
		</visual>
	</link>-->
	
	<!--================Robot Arm Joints=================-->

	<!--<joint name="bottom_base_to_top_base" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
		<parent link="bottom_base"/>
		<child link="top_base"/>
		<origin xyz="0 25 0" rpy="0 0 3.15"/>
	</joint>

	<joint name="top_base_to_first_segment" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
		<parent link="top_base"/>
		<child link="first_segment"/>
		<origin xyz="0 -15 0" rpy="1.5 0 0"/>
	</joint>-->

	<joint name="first_segment_to_second_segment" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
		<parent link="first_segment"/>
		<child link="second_segment"/>
		<origin xyz="-70 0 0" rpy="0 -1.5 0"/>
	</joint>

	<joint name="second_segment_to_claw_segment" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
		<parent link="second_segment"/>
		<child link="claw_segment"/>
		<origin xyz="-70 0 0" rpy="0 -1.5 0"/>
	</joint>

	<!--<joint name="claw_segment_to_first_claw" type="revolute">
		<axis xyz="0 -1 0"/>
		<limit effort="1000" lower="0" upper="1.57" velocity="0.5"/>
		<parent link="claw_segment"/>
		<child link="first_claw"/>
		<origin xyz="-75 13 -20" rpy="-1.6 3.1 1.57"/>
		<mimic joint="claw_segment_to_second_claw"/>
	</joint>

	<joint name="claw_segment_to_second_claw" type="revolute">
		<axis xyz="0 1 0"/>
		<limit effort="1000" lower="0" upper="1.57" velocity="0.5"/>
		<parent link="claw_segment"/>
		<child link="second_claw"/>
		<origin xyz="-75 -3 -22" rpy="-1.6 3.1 1.57"/>
	</joint>-->
</robot>
```

</p></details>
Although I create all the joints and links, most of them had to be commented out as when testing the arm only a max of
two servos could be used. This is further explained below.

Part III - Control the servo-motors from the robot's joint state
================================================================


<details><summary> **Robot Arm Code** </summary><p>

```cpp
//Include Packages
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h>

using namespace ros; //location

NodeHandle nh; //Node declare
Servo base; //Servo declare
Servo link1;
Servo link2;
Servo link3;
Servo claw;

#define BASE  2
#define LINK1 3
#define LINK2 4
#define LINK3 5
#define CLAW  6

void cb(const sensor_msgs::JointState& msg) //Uses 16 bit (UInt16) message as input, writes the data of
{                                     //the message to the servo.
  //int angle_base  = (int)(msg.position[0] * 180/3.14); //conversion from angle in radions to degrees
  //int angle_link1 = (int)(msg.position[1] * 180/3.14);
  int angle_link2 = (int)(msg.position[0] * 180/3.14);
  int angle_link3 = (int)(msg.position[1] * 180/3.14);
  //int angle_claw  = (int)(msg.position[3] * 180/3.14);
  
  //base.write(angle_base); //Write 0 - 180 to servo;
  //link1.write(angle_link1);
  link2.write(angle_link2);
  link3.write(angle_link3);
  //claw.write(angle_claw);
}

Subscriber<sensor_msgs::JointState> sub("joint_states", cb); //Creates sub of "servo" cb

void setup()
{
  nh.initNode(); //Initialise Node
  nh.subscribe(sub);
  
  //base.attach(2); //Attach to pin 9
  //link1.attach(3);
  link2.attach(4);
  link3.attach(5);
  //claw.attach(6);
void loop()
{
  nh.spinOnce();
  delay(1);
}
```

</p></details>

As can be see from the above code there is a lot of code commented out thats for other servos. This is due to the fact that
currently only two servos can be controlled. This is the case because only so much information can be carried through
the serial to the board, and with the message type being used, unnecessary space is taken up by things such as the names of
the joints and sections. These are things that arent needed therfore a node could be created to convert this information
into another message type such as an int array, therefore only sending over integer values and allowing more information
to be sent. However I havent yet been able to do this and therefore have only set it up to use two motors for the time being.

[![PartIII video](https://i.ytimg.com/vi/3TiCBh8qIgg/sddefault.jpg)](https://youtu.be/3TiCBh8qIgg)
