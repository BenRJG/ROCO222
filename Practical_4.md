Step 1 - Install the motor shield
---------------------------------
Installing the motor shield is a simple task, it simply involves lining the shield up with the board so that the pins from the shield
fit into the board.
![I_Part 1 Image 1][I_1_1]

[I_1_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_4/I_1_1.jpg?raw=true

Step 2 - Control a small hobby DC motor
---------------------------------------
The motor shield can be used to control a stepper motor. Below shows the setup of this:


The below example code can then be used to make the motor rotate:
```cpp
void setup() {
  //Setup Channel A
  pinMode(12, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() {
  //forward @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW); //Disengage the Brake for Channel A
  analogWrite(3, 255); //Spins the motor on Channel A at full speed
  delay(3000);
  digitalWrite(9, HIGH); //Eengage the Brake for Channel A
  delay(1000);
  
  //backward @ half speed
  digitalWrite(12, LOW); //Establishes backward direction of Channel A
  digitalWrite(9, LOW); //Disengage the Brake for Channel A
  analogWrite(3, 123); //Spins the motor on Channel A at half speed
  delay(3000);
  digitalWrite(9, HIGH); //Eengage the Brake for Channel A
  delay(1000);
}
```
From this code it can be seen that **pin 12** controls the direction, **pin 9** is the break, and **pin 3** controls the speed.
The direction of the speed can be changed by setting **12** **HIGH** or **LOW**.
The break can be **enabled** setting **9 HIGH** or dissabled setting it to **LOW**.
The speed can be adjusted from **0** to **255**

In this case the speed is set set to half speed. 
