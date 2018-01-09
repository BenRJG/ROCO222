Part II - Control a stepper motor
=================================
Step 1 - Wiring
---------------
Below is the scematics for the particular motor we used for this task:

![II_Step 1 Image 1][II_1_1]

[II_1_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_5/II_1_1.png?raw=true

Although the motor has 6 wires, for ourpurposes we only need the **Black**, **Green**, **Red** and **Blue**.
This is then connected to the board like so:

![II_Step 1 Image 2][II_1_2]

![II_Step 1 Image 3][II_1_3]

[II_1_2]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_5/II_1_2.jpg?raw=true
[II_1_3]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_5/II_1_3.jpg?raw=true

Step 2 - Initial Program
------------------------
Below is the initial program created to control the stepper motor:

<details><summary> Initial Code </summary><p>

```cpp
#define DIR_A 12
#define PWM_A 3

#define DIR_B 13
#define PWM_B 11

#define MAX 255

unsigned int speed = 5; //Min = 2

//Channel A is 12 & 9 ; Speed is 3
//Channel B is 13 & 8 ; Speed is 11

void turn(){
    analogWrite(PWM_A,MAX);
    analogWrite(PWM_B,0);
    delay(speed);
    
    analogWrite(PWM_A,0);
    analogWrite(PWM_B,MAX);
    delay(speed);
}

void rotate(unsigned int step){
   unsigned int i;
   for(i=0;i<(step);i++){
    digitalWrite(DIR_A,HIGH);
    digitalWrite(DIR_B,HIGH);
    
    turn();
    
    digitalWrite(DIR_A,LOW);
    digitalWrite(DIR_B,LOW);
    
    turn();
   }
}

void setup() {
    //Setup Channel A
    pinMode(DIR_A, OUTPUT);
    
    pinMode(DIR_B, OUTPUT);
}



void loop() {
    //forward at full speed
  rotate(50);
  while(1);
    
}
```

</p></details>

Step 3 - Programming of modes
-----------------------------
### Full-step mode
|   | A | B |!A |!B |
|---|---|---|---|---|
| 1 | 1 | 0 | 0 | 0 |
| 2 | 0 | 1 | 0 | 0 |
| 3 | 0 | 0 | 1 | 0 |
| 4 | 0 | 0 | 0 | 1 |

<details><summary> Full-Step Code </summary><p>
  
```cpp
#define DIR_A 12
#define PWM_A 3

#define DIR_B 13
#define PWM_B 11

#define MAX 255

unsigned int speed = 5; //Min = 2

//Channel A is 12 & 9 ; Speed is 3
//Channel B is 13 & 8 ; Speed is 11

void full_step(){
    digitalWrite(DIR_A,HIGH);
    digitalWrite(DIR_B,HIGH);
    
    analogWrite(PWM_A,MAX);
    analogWrite(PWM_B,0);
    delay(speed);
    
    analogWrite(PWM_A,0);
    analogWrite(PWM_B,MAX);
    delay(speed);

    digitalWrite(DIR_A,LOW);
    digitalWrite(DIR_B,LOW);

    analogWrite(PWM_A,MAX);
    analogWrite(PWM_B,0);
    delay(speed);
    
    analogWrite(PWM_A,0);
    analogWrite(PWM_B,MAX);
    delay(speed);
}

void rotate(unsigned int step){
   unsigned int i;
   for(i=0;i<(step);i++){
      full_step();
   }
}

void setup() {
    //Setup Channel A
    pinMode(DIR_A, OUTPUT);
    
    pinMode(DIR_B, OUTPUT);
}



void loop() {
    //forward at full speed
  rotate(1);
    
}
```
  
<p></details>

Full-step mode activates two opposing magnets at a time, which can be seen from the above
truth table, A is enabled, then disabled, then B is enabled then disabled, A is inverted
then disabled and B is inverted then disabled and this repeats causing the motor to rotate.
However this causes a lot of vibration. This mode also results in low torque.


[![II_Step 3 Video 1][II_3_1(T)]][II_3_1(V)]

[II_3_1(T)]: https://i.ytimg.com/vi/RJmsSzU62jI/sddefault.jpg
[II_3_1(V)]: https://youtu.be/RJmsSzU62jI

### Double-step mode
|   | A | B |!A |!B |
|---|---|---|---|---|
| 1 | 1 | 1 | 0 | 0 |
| 2 | 0 | 1 | 1 | 0 |
| 3 | 0 | 0 | 1 | 1 |
| 4 | 1 | 0 | 0 | 1 |

<details><summary> Double-Step Code </summary><p>
  
```cpp
#define DIR_A 12
#define PWM_A 3

#define DIR_B 13
#define PWM_B 11

#define MAX 255

unsigned int speed = 5; //Min = 2

//Channel A is 12 & 9 ; Speed is 3
//Channel B is 13 & 8 ; Speed is 11

void double_step(){
    digitalWrite(DIR_A,HIGH);
    digitalWrite(DIR_B,HIGH);
    
    analogWrite(PWM_A,MAX);
    analogWrite(PWM_B,MAX);
    delay(speed);

    digitalWrite(DIR_A,LOW);
    delay(speed);
    digitalWrite(DIR_B,LOW);
    delay(speed);
    digitalWrite(DIR_A,HIGH);
    delay(speed);
}

void rotate(unsigned int step){
   unsigned int i;
   for(i=0;i<(step);i++){
      double_step();
   }
}

void setup() {
    //Setup Channel A
    pinMode(DIR_A, OUTPUT);
    
    pinMode(DIR_B, OUTPUT);
}



void loop() {
    //forward at full speed
  rotate(1);
    
}
```
  
</p></details>

Double-step mode always has all the magnets enabled, therefore increasing the amount
of torque of the motor. The process of this can be seen from the above table.


[![II_Step 3 Video 2][II_3_2(T)]][II_3_2(V)]

[II_3_2(T)]: https://i.ytimg.com/vi/UdCte2OhZEI/sddefault.jpg
[II_3_2(V)]: https://youtu.be/UdCte2OhZEI

### Half-step mode
|   | A | B |!A |!B |
|---|---|---|---|---|
| 1 | 1 | 0 | 0 | 0 |
| 2 | 1 | 1 | 0 | 0 |
| 3 | 0 | 1 | 0 | 0 |
| 4 | 0 | 1 | 1 | 0 |
| 5 | 0 | 0 | 1 | 0 |
| 6 | 0 | 0 | 1 | 1 |
| 7 | 0 | 0 | 0 | 1 |
| 8 | 1 | 0 | 0 | 1 |

<details><summary> Half-Step Code </summary><p>
  
```cpp
#define DIR_A 12
#define PWM_A 3

#define DIR_B 13
#define PWM_B 11

#define MAX 255

unsigned int speed = 5; //Min = 2

//Channel A is 12 & 9 ; Speed is 3
//Channel B is 13 & 8 ; Speed is 11

void half_step(){
    digitalWrite(DIR_A,HIGH);
    digitalWrite(DIR_B,HIGH);
    
    analogWrite(PWM_A,MAX);
    analogWrite(PWM_B,0);
    delay(speed);
    
    analogWrite(PWM_B,MAX);
    delay(speed);

    analogWrite(PWM_A,0);
    delay(speed);

    digitalWrite(DIR_A,LOW);
    analogWrite(PWM_A,MAX);
    delay(speed);
    
    digitalWrite(DIR_B,LOW);
    analogWrite(PWM_B,0);

    analogWrite(PWM_B,MAX);
    delay(speed);
    
    analogWrite(PWM_A,0);
    delay(speed);

    digitalWrite(DIR_A,HIGH);
    analogWrite(PWM_A,MAX);
}

void rotate(unsigned int step){
   unsigned int i;
   for(i=0;i<(step);i++){
      half_step();
   }
}

void setup() {
    //Setup Channel A
    pinMode(DIR_A, OUTPUT);
    
    pinMode(DIR_B, OUTPUT);
}



void loop() {
    //forward at full speed
  rotate(1);
    
}
```

</p></details>

Half step combines the previous two modes, alternating between 2 and 4 magnets enabled.
This increases the angular resolution.

### Micro-step mode

<details><summary> Micro-Step Code </summary><p>
  
```cpp
#define DIR_A 12
#define PWM_A 3

#define DIR_B 13
#define PWM_B 11

#define MAX 255

unsigned int speed = 5; //Min = 2

//Channel A is 12 & 9 ; Speed is 3
//Channel B is 13 & 8 ; Speed is 11

void WriteValue(int value, int chanAnalog, int chanDigit) //Not able to create -255 voltage, changes to 255, and sets HIGH to LOW
{
  int vaueAbs = abs(value); //Get absolute value
  analogWrite(chanAnalog, value); //Write absolue value as pwm
  if(value > 0){
    digitalWrite(chanDigit,HIGH);
  }
  else{
    digitalWrite(chanDigit,LOW);
  }
}

//Micro_Step//////////////////////////////////////////////////////////
int microsteps = 200; //Howmany steps for one rotation (200)
float amp = 255; //Max output
int pulseDelay = 10; //microseconds between write
int a[200];
int b[200];
int idxG;

void micro_step_INIT(){
  int idx;
  idxG=0;

  for (idx = 0; idx < microsteps; idx++)
  {
    a[idx] = amp * sin(idx * 2  * PI / microsteps); //Builds Sin and Cos tables (2*pi for radians)
    b[idx] = amp * cos(idx * 2 * PI / microsteps);
  }
}

void setup() {
    //Setup Channel A
    Serial.begin(9600);
    
    pinMode(DIR_A, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    
    pinMode(DIR_B, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    digitalWrite(8, LOW);
    digitalWrite(9, LOW);

    micro_step_INIT();
}



void loop() {
    //forward at full speed
  WriteValue(a[idxG], PWM_A, DIR_A);
  WriteValue(b[idxG], PWM_B, DIR_B);
  idxG++;
  
  if(idxG == microsteps){
    idxG = 0;
  }

    delayMicroseconds(pulseDelay);
}
```

</p></details>

Micro-step mode gradually increases and reduces the power of the magnets in order to greatly improve
the operation of the motor and make it vibrate a lot less whilst maintaining torque. This will make
the motor smoothly move between each position rather than suddenly moving.

[![II_Step 3 Video 3][II_3_3(T)]][II_3_3(V)]

[II_3_3(T)]: https://i.ytimg.com/vi/uSPSt5KofKA/sddefault.jpg
[II_3_3(V)]: https://youtu.be/uSPSt5KofKA
