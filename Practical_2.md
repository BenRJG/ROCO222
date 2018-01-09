Table of Contents
=================
* [Part I - Build a brushed DC electrical motor](#part-i)
  1. [Building the commutator](#i-step-1)
  2. [Add support Shaft](#i-step-2)
  3. [Winding the armature coil](#i-step-3)
  4. [Building the shaft support and magnet brackets](#i-step-4)
  5. [Build the baseplate](#i-step-5)
* [Part II - Hack into a robot](#part-ii)
  1. [Finish the motor](#ii-step-1)
  2. [Test the motor](#ii-step-2)
* [Part III - A better DC motor](#part-iii)
  1. [Building the commutator](#iii-step-1)
  2. [Add support Shaft](#iii-step-2)
  3. [Winding the armature coil](#iii-step-3)
  4. [Building the shaft support and magnet brackets](#iii-step-4)
  5. [Build the baseplate](#iii-step-5)

Practical 2: Build a DC motor
==============================================
Part I - Build a brushed DC electrical motor<a name = "part-i">
------------------------------------------
The aim of this part is to create a DC motor. For this we have:
* Tape
* Paper Clips
* 10m Copper Wire
* 2 Nails

### Step 1 - Building the commutator<a name = "i-step-1">
The commutator is the moving part of the motor that will rotate within the magnetic field. This was started by placing copper
tape on opposite sides of the cork, ensuring to leave a gab between each strip. This is needed as the tape will later be connected
to two ends of the copper wire, and will be used to reverse the current.

_The image below shows the commutator with 1 long strip of copper tape rather than 2 pieces of tape before it was created_
![Step 1 Image 1][I_1_1]

[I_1_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%201%20-%201.jpg?raw=true "Commutator with copper tape"

### Step 2 - Add support Shaft<a name = "i-step-2">
A support shaft for the commutator was made using two nails placed into the two ends of the cork. This will allow the commutator
to move freely when placed on a support.
![Step 2 Image 1][I_2_1]

[I_2_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%202%20-%201.jpg?raw=true

Tape was then added around the copper tape to better hold it in place:
![Step 2 Image 2][I_2_2]

[I_2_2]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%202%20-%202.jpg?raw=true

### Step 3 - Winding the armature coil<a name = "i-step-3">
This was the step that took the longest! A coil was wraped lengthways around the cork forming a coil.
It was recomended<a name = "I-step-2"> for this that the coil had 60 turns minimum and we managed aroudn 139 turns (give or take 2 or 3
due to the monotonus of counting the turns whilst wrapping).
![Step 3 Image 1][I_3_1]

Both ends of the wire were left free at the top (end with copper tape), then each end was soldered onto one of the two pieces of
copper tape. This is how the current will flow through the coil.
Due to the enamal on the copper wire preventing elctricity from conduction through it, sandpaper was needed to sand it off
the ends before soldering.

The resistance of the coild was measured at =~7 ohms.
![Step 3 Image 2][I_3_2]

[I_3_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%203%20-%201.jpg?raw=true
[I_3_2]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%203%20-%202.jpg?raw=true
### Step 4 - Building the shaft support and magnet brackets<a name = "i-step-4">
The shaft support was made by using two paper clips, and stretching out one end, and curling it round into a small loop as shown
in the image. This will later create an ideal support for the twon nails used as a support shaft.

The magnet bracket was made by simply bending the paper clip into an L-shape.
![Step 4 Image][I_4_1]

[I_4_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%204%20-%201.jpg?raw=true

### Step 5 - Build the baseplate<a name = "i-step-5">
Firstly the wooden base needed to be evenly devided horizontally and vertically (which we didn't need to do as it was already
done). This allowed the paper clips to be equally fastened.

The two support paper clips were screwed into the board lengthways along the board. The commutator could then be rested between
these.
The two magnet brackets were attached width ways, at a 90° to the commutator. Two of each of the magnets were then put onto the
brackets, which could support themselves due to their magnetic strength. These were attached so that all the magnets were in the
same direction and attracting to one another.
![Step 5 Image][I_5_1]

[I_5_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/I%20-%205%20-%201.jpg?raw=true

Part II - Hack into a robot<a name = "part-ii">
-----------------------------------------------
### Step 1 - Finish the motor<a name = "ii-step-1">
In order to apply power to the commutator, cables currently be directly attached to the coil as this would obviously prevent the
commutator from rotating. Therefore whatever wire was used had to touch, but not be fixed to, the commutator. The copper tape that
was used gives a greater surface are of contact, to help remain contact to the power.

Firstly we attempted to use wire to act as "brushes" for the motor, by connecting a cable to the power supply and to a thinner wire
with crocodile clips. As the wire was thin it had some degree of freedom to be able to use as brushes.

The powersupply had a current limit of 2A. This was to prevent the current exceding this as this could cause the wire to overheat and
eather melt or damage other components. Although the coil acts as a resistor, it only has a small resistance, and therefore only
limits the current a small amount and therefore has the potential to overheat.

### Step 2 - Test the motor<a name = "ii-step-2">
When testing the motor, we did manage to get the commutator to rotate. However as the wires had thin ends, they were prone to getting
caught on the copper wire and on the copper tape. This therefore would occasionally make the motor stop functioning.

[![Wires for Brushes][II_2_1(T)]][II_2_1(V)]

[II_2_1(T)]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/Thumbnails/II_2_1.png?raw=true
[II_2_1(V)]: https://youtu.be/h_XL3oUzkJ0

To combat this we decided to use copper tape instead, as not only were they more flexible and therefore less likely to catch on
anything, they also have a large surface area, therefore increasing the contact surface area. Using this proved to be much more
efficient.

[![Tape for Brushes][II_2_2(T)]][II_2_2(V)]

[II_2_2(T)]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/Thumbnails/II_2_2.png?raw=true
[II_2_2(V)]: https://youtu.be/ZZyAr5DxDUE

It can also be seen from the video above that the higher the voltage, the higher the RPM of the motor.

After we got the motor working, we then tested how the motor acted when one of the magnets was rotated the opposite way, therefore
repelling each other. This seemed to create a conflict with the motor, and caused it to vibrate viciously as it rapidly tried to
rotate both ways at the same time.

[![Opposing Magnets][II_2_3(T)]][II_2_3(V)]

[II_2_3(T)]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/Thumbnails/II_2_3.png?raw=true
[II_2_3(V)]: https://youtu.be/hRVDDwe9pTM

When the motor was functioning, it could be seen that on the powersupply both the current *and* the voltage were fluctuating:

[![Alternating Voltage][II_2_4(T)]][II_2_4(V)]

[II_2_4(T)]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/Thumbnails/II_2_4.png?raw=true
[II_2_4(V)]: https://youtu.be/w-qEDNoe-Ts

Part III - A better DC motor<a name = "part-iii">
-------------------------------------------------
When developing and improving the motor we decided to 3D print the parts.

### Step 1 - Building the commutator<a name = "iii-step-1">
The commutator we made for this we designed it to use 2 coils as apposed to the previous 1. This allowed for more torque to be generate
within the motor. This allows for one coil to always be cutting accross the magnetic field and therefore generate torque to rotate.

Next time however we would choose to use 3 or more coils, as the would ensure that a coil is always at an angle, in the current case of
only 2 coils it often needs a little push to get going.

### Step 2 - Add support Shaft<a name = "iii-step-2">
The support shaft for this was 3D printed as part of the commutator. This proved to be difficult practically when printing the
commutator.
  
When doing this again we would use a different material as the support shaft through the commutator as this would help increase the magnetic field and therefore increase the torque and efficiency of the motor.

### Step 3 - Winding the armature coil<a name = "iii-step-3">
For this commutator we only used around 50 turns for the coils. The proved to be very inefficent and therefore the motor is increadbly weak. Next time would add many more turns to this (as witht the original with around 130 turns).

![Step 3 Image 1][III_3_1]

[III_3_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/III_3_1.JPG?raw=true "New Motor Design"

### Step 4 - Building the shaft support and magnet brackets<a name = "iii-step-4">
We did want to 3D print the components of this however due to technical issues with the printer we were using we were unable to do so.
We did, however, mangage to print most of the brackets to support the the shaft.
*We then later printed the support shafts for the magnets when we had time*

![Step 4 Image 1][III_4_1]

[III_4_1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images2/Practical_2/III_4_1.JPG?raw=true "Added Supports"


On the brackets we used bearings to allow the motor to smoothly rotate on its axis. The magnets, due to the lack of printing the parts,
had to remain supported by the paper clips from the original design.

### Step 5 - Build the baseplate<a name = "iii-step-5">
Everything was re assembled onto the same wooden baseplate as before. Two strips of copper tape were used as brushes pressed against the
commutator.
If I was to redesign this I would create brackets to hold the brushes closer and firmer to the commutator.
