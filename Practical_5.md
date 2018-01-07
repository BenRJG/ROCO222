Part II - Control a stepper motor
=================================
Step 1 - Wiring
---------------
Below is the scematics for the particular motor we used for this task:

***schematic***

Although the motor has 6 wires, for ourpurposes we only need the **Black**, **Green**, **Red** and **Blue**.
This is then connected to the board like so:
***Board setup***

Step 2 - Initial Program
------------------------
Below is the initial program created to control the stepper motor:
```cpp
//Code snippet
bla bla
```

Step 3 - Programming of modes
-----------------------------
### Full-step mode
|   | A | B |!A |!B |
|---|---|---|---|---|
| 1 | 1 | 0 | 0 | 0 |
| 2 | 0 | 1 | 0 | 0 |
| 3 | 0 | 0 | 1 | 0 |
| 4 | 0 | 0 | 0 | 1 |

Full-step mode activates two opposing magnets at a time, which can be seen from the above
truth table, A is enabled, then disabled, then B is enabled then disabled, A is inverted
then disabled and B is inverted then disabled and this repeats causing the motor to rotate.
However this causes a lot of vibration. This mode also results in low torque.

### Double-step mode
|   | A | B |!A |!B |
|---|---|---|---|---|
| 1 | 1 | 1 | 0 | 0 |
| 2 | 0 | 1 | 1 | 0 |
| 3 | 0 | 0 | 1 | 1 |
| 4 | 1 | 0 | 0 | 1 |

Double-step mode always has all the magnets enabled, therefore increasing the amount
of torque of the motor. The process of this can be seen from the above table.

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

half step combines the previous two modes, alternating between 2 and 4 magnets enabled.
This increases the angular resolution.

### Micro-step mode
Micro-step mode gradually increases and reduces the power of the magnets in order to greatly improve
the operation of the motor and make it vibrate a lot less whilst maintaining torque. This will make
the motor smoothly move between each position rather than suddenly moving.
