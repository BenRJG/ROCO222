# What is Markdown?
Markdown is basically a way to create a text document, then edit and styalize it using syntax for display on the web.
For example using '*Hello*' in the text editor will look like *Hello* on the web.

This allows for the text to be edited by the created in any way desired without having to worry about selecting fonts or adding titles.

## Key Syntax
* `*Italic*` or `_Italic_` = *Italic*
* `**Bold**` or `__Bold__` = **Bold**
* `#Header` will create a header like the top of the page
* `##Header` will create a header like the top of this list
* `###Header` will make a third level header, and so on.
* `>This symbol can be used to make quotes` >This symbol can be used to make quotes
* Using the ``` Symbol allows for *Italic* to be shown as `*Italic*` (``*Italic*``)
* Create lists using `*`, `1.` or `-`:

* One
* Two
	* Three
	* Four

1. One
2. Two
	1. Three
	2. Four

- One
- Two
	- Three
	- Four

# Using the Terminal
* `ls` shows the availabale folders within the currently selected directory.
* `cd /tmp` this takes the user to the temporary directory of '/tmp'
* `cd $HOME` takes us aback to the Home directory. Using the $ at the start uses the HOME as a representative of something else, in this case '/home/student'
* `mkdir` is used to make directories
* `echo "Hello"` will display Hello in the terminal
* `echo "Hello" > hello.md` produces a '.md' file with the text 'Hello'
* `cat hello.md` displays the text within the file 'hello.md'
* `cp hello.md hello-again.md` copies the file 'hello.md' and renames it to 'hello-again.md'
* `mv hello-again.md hello-hello.md` moves and renames the file.
* `rm hello.md` removes the file
* `rm -rf`
* `cat /proc/cpuinfo` displays the text contents of the file 'cpuinfo'

# Using Git
Git is used to track the changes within a particular file. Firstly the '.git/' file needs to be created using 'git init'. From here on this can be used to log the changes within a given file.

Once git has been initialised, a name and email needs to be set using 'git config user.name "Firstname Surname"' and 'git config user.email "email"
So in my case it'd be 'git config user.name "Ben Gordon' and 'git config user.name `ben.gordon@students.plymouth.ac.uk`'

Then the journal file can be added to the git folder using 'git add journal.md', then create a new commit using 'git commit'. When doing this a short (usually less than 72 characters) summary of the changes should be typed on what has been added/changed to the file.

# Hacking the Robot
After browsing round many websites, we finally figured out how to hack the robot and make it talk!

## Connecting to chapman
### Finding the IP
Firstly we had to connect to the chapman robot.

To do this we first went to chapman.local in the web browser. This brought us to a configuration page for chapman.

From this we could find the robot's IP address by going to the **Network Settings** and clicking on **Wired**. This showed us that the IP was **192.167.0.184**.

### Connecting
Once we had the IP, we could then connect to chapman through the terminal by typing `ssh nao@192.167.0.184`. This connects us to the robot to be able to navigate files and use python to control the speech.

## Making Chapman talk
To make him talk was a simple task. first we typed `python` into the console, which opened a python console to input code.

Then the following commands were entered:
* `from naoqi import ALProxy`
* `tts = ALProxy("ALTextToSpeech", "localhost", 9559)` - this enables the text to speach and assigns
.. it `tts`
* finally Chapman can be made to say anything by inputing `tts.say("Input Text Here")`

# Building a DC motor
## Part I - Building a brushed DC motor
The aim of this part is to create a DC motor. For this we have:
* Cork
* Copper Tape
* Paper Clips
* 10m Copper Wire
* 2 nails

### Step 1 - Building the commutator
The commutator is the moving part of the motor that will rotate within the magnetic field. This was started by placing copper
tape on opposite sides of the cork, ensuring to leave a gab between each strip. This is needed as the tape will later be connected
to two ends of the copper wire, and will be used to reverse the current.
Electrical tape was then used to hold it in place.

### Step 2 - Support Shaft
A support shaft for the commutator was made using two nails placed into the two ends of the cork. This will allow the commutator
to move freely when placed on a support.

### Step 3 - Winding the armature coil
This was the step that took the longest! A coil was wraped lengthways around the cork forming a coil.
It was recomended for this that the coil had 60 turns minimum and we managed aroudn 139 turns (give or take 2 or 3 due to the
monotonus of counting the turns whilst wrapping).
Both ends of the wire were left free at the top (end with copper tape), then each end was soldered onto one of the two pieces of
copper tape. This is how the current will flow through the coil.
Due to the enamal on the copper wire preventing elctricity from conduction through it, sandpaper was needed to sand it off
the ends before soldering.

The resistance of the coild was measured at =~7 ohms.

### Step 4 - Building the shaft support & magnet brackets
The shaft support was made by using two paper clips, and stretching out one end, and curling it round into a small loop as shown
in the image. This will later create an ideal support for the twon nails used as a support shaft.

The magnet bracket was made by simply bending the paper clip into an L-shape.

### Step 5 - Assembly
Firstly the wooden base needed to be evenly devided horizontally and vertically (which we didn't need to do as it was already
done). This allowed the paper clips to be equally fastened.

The two support paper clips were screwed into the board lengthways along the board. The commutator could then be rested between
these.
The two magnet brackets were attached width ways, at a 90° to the commutator. Two of each of the magnets were then put onto the
brackets, which could support themselves due to their magnetic strength. These were attached so that all the magnets were in the
same direction and attracting to one another.
