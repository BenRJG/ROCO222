Part I - Introduction
=====================================
Step 1 - Circuit Diagram
-----------------------------
Below is the required circuit diagram for this component:
***Insert circuit diagram***

This will be used in order the measure the speed of the motor. An IR light will be used to shine into a reciever, this is blocked by a
disk attached to the end of the motor. As the motor rotates, a slit in the disk will allow the reciever to detect the light and send
a pulse that can be used to count the revolutions.

Step 2 - Build the IR LED light source
--------------------------------------
The IR source and reviever were both tested and functional bedfore being used within the circuit.

Step 3 - Build the light detector
---------------------------------
The light detector we built is shown below:
***Light Detector Pic***

Although this was fully functional (as shown below) after being built, as I go into later on we ran into issues testing.
***Light Detector Pic***

Step 4 - Place an encoder disc on the motor
-------------------------------------------
The final step of assembly is to attach a disc to the setup, the final product is shown below:
***Final Pic***


Part II - Arduino
=================
Now the next step is to configure the software to measure the speed of the motor using the hardware.

Step 1 - Launch the Arduino IDE
-------------------------------
Firstly the setup of the arduino needs to be checked and ensure the board is correctly configured:
***Setup***

Step 2 - Connect your encoder
-----------------------------
Next is to connect the encoder to the board. This will then allow the arduino to use the pulse from the encoder to measure the sepeed.
***pic of conection***
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
I also added some extra code to output to a terminal when triggered. This shows it working below:
***trigger video***

Step 3 - Calculate the angular velocity of your motor
------------------------------------------------------
This is where we began to have issues as although the encoder worked perfectly fine, when the motor was turned on the counter began
to rapidly increase for no apparent reason. This is shown below:
***Video example***
