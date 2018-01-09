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

![II_Part x Image 1][II_x_1]

[II_x_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_6/II_x_1.jpg?raw=true

The directional input is used to select the position of the motor. A diagram below shows this:

![II_Part x Image 2][II_x_2]

[II_x_2]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_6/II_x_2.PNG?raw=true

From the diagram:
* **0.7ms - 1.5ms** varies the angle from **0° <= θ <90°** *(Clockwise from center)*
* **1.5ms** sets the angle to **θ = 90°** *(Center)*
* **1.5ms - 2.0ms** varies the angle from **90° < θ <= 180°** *(Counter clockwise from center)*

Step 1 - Control an RC servo
----------------------------
Firstly the motor needs to be connected to the board. To make this easier I connected it to
a breadboard first, then that to the board.

I then created some code in order to make the motor move over its entire range of about 0.2Hz.

<details><summary> Sweep Code </summary><p>
  
```cpp
#include <Servo.h>

#define PWM_A 5

Servo servo_A;

//WAVE GEN
int steps = 200;
float pos = 90;
int pulse = 25;
int sine[200];
int idxG;

void sineGen()
{
  int idx;
  idxG=0;

  for(idx = 0; idx < steps; idx++)
  {
    sine[idx] = (pos * sin(idx * 2 * PI / steps))+90;
  }
}

void setup() {
  // set the digital pin as output:
  servo_A.attach(PWM_A);

  sineGen();
}

void loop() {
  servo_A.write(sine[idxG++]);
  if(idxG == 200){idxG = 0;}
  delay(pulse);
}
```

</p></details>


[![II_Step 1 Video 1][II_1_1(T)]][II_1_1(V)]

[II_1_1(T)]: https://i.ytimg.com/vi/Q1mAJHBqmKA/sddefault.jpg
[II_1_1(V)]: https://youtu.be/Q1mAJHBqmKA

Step 2 - Control the servo with a potentiometer
-----------------------------------------------
When coding the servo motor to be controlled with a potentiometer, I managed to get some
functional code, however the motor didnt correctly function when using it, whereas it would
suddenly increase its angle after when the dial is rotated a certain amount, rather than
gradually changing throughout.

<details><summary> Servo Control Code </summary><p>
  
```cpp
#include <Servo.h>

#define PWM_A 5

#define dial 0

Servo servo_A;

int value = 0;

void dialControl(){
  value = analogRead(dial);
  
  Serial.print(value);
  Serial.print(" => ");
  value = map(value,0,1023,0,180);
  Serial.print(value);
  Serial.print("\n");

  servo_A.write(value);
  delay(15);
}

void setup() {
  // set the digital pin as output:
  Serial.begin(9600);
  
  servo_A.attach(PWM_A);

  pinMode(dial,INPUT); 

}

void loop() {
  dialControl();
}
```

</p></details>



[![II_Step 2 Video 1][II_2_1(T)]][II_2_1(V)]

[II_2_1(T)]: https://i.ytimg.com/vi/bU45XKqbKYc/sddefault.jpg
[II_2_1(V)]: https://youtu.be/bU45XKqbKYc
