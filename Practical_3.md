Part I - Introduction
=====================================
Step 1 - Circuit Diagram
-----------------------------
Below is the required circuit diagram for this component:
![I_Step 1 Image 1][I_1_1]

[I_1_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_3/I_1_1.PNG?raw=true

This will be used in order the measure the speed of the motor. An IR light will be used to shine into a reciever, this is blocked by a
disk attached to the end of the motor. As the motor rotates, a slit in the disk will allow the reciever to detect the light and send
a pulse that can be used to count the revolutions.

Step 2 - Build the IR LED light source
--------------------------------------
The IR source and reviever were both tested and functional bedfore being used within the circuit.

Step 3 - Build the light detector
---------------------------------
The light detector we built is shown below:

![I_Step 3 Image 1][I_3_1]

[I_3_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_3/I_3_1.jpg?raw=true

Although this was fully functional after being built, as I go into later on we ran into issues testing.

Step 4 - Place an encoder disc on the motor
-------------------------------------------
The final step of assembly is to attach a disc to the setup, the final product is shown below:

![I_Step 4 Image 1][I_4_1]

[I_4_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_3/I_4_1.JPG?raw=true


Part II - Arduino
=================
Now the next step is to configure the software to measure the speed of the motor using the hardware.

Step 1 - Launch the Arduino IDE
-------------------------------
Firstly the setup of the arduino needs to be checked and ensure the board is correctly configured:

![II_Step 1 Image 1][II_1_1]

![II_Step 1 Image 2][II_1_2]

[II_1_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_3/II_1_1.png?raw=true
[II_1_2]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_3/II_1_2.png?raw=true

Step 2 - Connect your encoder
-----------------------------
Next is to connect the encoder to the board. This will then allow the arduino to use the pulse from the encoder to measure the sepeed.

Then the following code can be used to test it:
```cpp
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  // configure the interrupt call-back: blink is called everytime the pin
  // goes from low to high.
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
}

void loop() {
  digitalWrite(ledPin, state);
}

void blink() {
  state = !state;
}
```
I also added some extra code to output to a terminal when triggered. 

Step 3 - Calculate the angular velocity of your motor
------------------------------------------------------
This is where we began to have issues as although the encoder worked perfectly fine, when the motor was turned on the counter began
to rapidly increase for no apparent reason. This is shown below:

