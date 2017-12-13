Robotic Arm Mini-project Part 2 - ROS
=====================================
Step 2 - First steps with ROS
-----------------------------
When using ROS, "roscore" first needs to be launched:

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
First wass using rostopic

```console
ben@b-gordon:~$ rostopic list
/rosout
/rosout_agg
```

As can be seen this displays two files.
According to the ROS wiki, /rosout is used for log reporting. It'll record the messages and store them in a text log files and
rebroadcasts them on /rosout_agg.

To publish on a new topic I use the following command:
```console
ben@b-gordon:~$ rostopic pub /test std_msgs/String "Hello"
publishing and latching message. Press ctrl-C to terminate
```

Now when listing rostopic 3 items can be seen:
```console
ben@b-gordon:~$ rostopic list
/rosout
/rosout_agg
/test
```
The bottom most one can be seen to be the topic whe just created "/test".


Upon opening a new terminal, the console messgaes can be seen from /test:
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
