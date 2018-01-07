Part I - The Robotic Arm mini-project
=====================================
For this mini-project the aim is to create a functional robotic arm.
The arm needs to have a *minimum* of 2 degrees of freedom using two servo motors.
The rough schedule we were given is shwon below:

* **Week 1:** Control of two servo-motors with the Arduino; use of potentiometers to control them
* **Week 2:** Tutorial led by Jake on 3D design for effective 3D printing
* **Week 3:** Design of the arm continued; arm assembly
* **Week 4:** ROS on Arduino; control of the servo using ROS
* **Week 5:** 3D model of the arm, visualisation of the arm in RViz
* **Week 6:** Finalisation of arms; if time permit, 3D motion plannin

Part II - Arduino RC servo control
==================================
For this project we need to use servo motors, therefore the first step is to make it work.
The servo connection consists of three inputs, the **brown** wire is ground, **red** is voltage
and **yellow** is the directional input.

The directional input is used to select the position of the motor. A diagram below shows this:
**Diagram pic**

From the diagram:
* **0.7ms - 1.5ms** varies the angle from **0° <= θ <90°** *(Clockwise from center)*
* **1.5ms** sets the angle to **θ = 90°** *(Center)*
* **1.5ms - 2.0ms** varies the angle from **90° < θ <= 180°** *(Counter clockwise from center)*

Step 1 - Control an RC servo
----------------------------
Firstly the motor needs to be connected to the board. To make this easier I connected it to
a breadboard first, then that to the board.
***Connection Pic***

I then created some code in order to make the motor move over its entire range of about 0.2Hz.
***Code snippet***
***Movement video***

Step 2 - Control the servo with a potentiometer
-----------------------------------------------
When coding the servo motor to be controlled with a potentiometer, I managed to get some
functional code, however the motor didnt correctly function when using it, whereas it would
suddenly increase its angle after when the dial is rotated a certain amount, rather than
gradually changing throughout.
***Code Snippet***
***Video example***

Step 3 - A robot arm mock-up
----------------------------
