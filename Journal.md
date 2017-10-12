# Table of Contents
1. [What is Markdown?](#what-is-markdown?)
	* [Key Syntax](#key-syntax)
2. [Using the Terminal](#using-the-terminal)
3. [Using Git](#using-git)
4. [Hacking the Robot](#hacking-the-robot)
	* [Connecting to chapman](#connecting-to-chapman)
		* [Finding the IP](#finding-the-ip)
		* [Connecting](#connecting)
	* [Making Chapman talk](#making-chapman-talk)
5. [Building a DC motor](#building-a-dc-motor)
	* [Part I - Building a brushed DC motor](#part-i)
		* [Step 1 - Building the commutator](#i-step-1)
		* [Step 2 - Support Shaft](#i-step-2)
		* [Step 3 - Winding the armature coil](#i-step-3)
		* [Step 4 - Building the shaft support & magnet brackets](#i-step-4)
		* [Step 5 - Assembly](#i-step-5)

# What is Markdown?<a name = "what-is-markdown?">
Markdown is basically a way to create a text document, then edit and styalize it using syntax for display on the web.
For example using '*Hello*' in the text editor will look like *Hello* on the web.

This allows for the text to be edited by the created in any way desired without having to worry about selecting fonts or adding titles.

## Key Syntax<a name = "key-syntax">
* `*Italic*` or `_Italic_` = *Italic*
* `**Bold**` or `__Bold__` = **Bold**
* `#Header` will create a header like the top of the page
* `##Header` will create a header like the top of this list
* `###Header` will make a third level header, and so on.
* `>This symbol can be used to make quotes` >This symbol can be used to make quotes
* Using the `'` Symbol allows for *Italic* to be shown as `*Italic*` (`'*Italic*'`)
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

# Using the Terminal<a name = "using-the-terminal">
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
* `cat /proc/cpuinfo` displays the text contents of the filhacking-the-robot'

# Using Git<a name = "using-git">
Git is used to track the changes within a particular file. Firstly the '.git/' file needs to be created using 'git init'. From here on this can be used to log the changes within a given file.

Once git has been initialised, a name and email needs to be set using 'git config user.name "Firstname Surname"' and 'git config user.email "email"
So in my case it'd be 'git config user.name "Ben Gordon' and 'git config user.name `ben.gordon@students.plymouth.ac.uk`'

Then the journal file can be added to the git folder using 'git add journal.md', then create a new commit using 'git commit'. When doing this a short (usually less than 72 characters) summary of the changes should be typed on what has been added/changed to the file.

# Hacking the Robot<a name = "hacking-the-robot">
After browsing round many websites, we finally figured out how to hack the robot and make it talk!

## Connecting to chapman<a name = "connecting-to-chapman">
### Finding the IP<a name = "finding-the-ip">
Firstly we had to connect to the chapman robot.

To do this we first went to chapman.local in the web browser. This brought us to a configuration page for chapman.

From this we could find the robot's IP address by going to the **Network Settings** and clicking on **Wired**. This showed us that the IP was **192.167.0.184**.

### Connecting<a name = "connecting">
Once we had the IP, we could then connect to chapman through the terminal by typing `ssh nao@192.167.0.184`. This connects us to the robot to be able to navigate files and use python to control the speech.

## Making Chapman talk<a name = "making-chapman-talk">
To make him talk was a simple task. first we typed `py<a name = "finding-the-ip">thon` into the console, which opened a python console to input code.

Then the following commands were entered:
* `from naoqi import ALProxy`
* `tts = ALProxy("ALTextToSpeech", "localhost", 9559)` - this enables the text to speach and assigns
.. it `tts`
* finally Chapman can be made to say anything by inputing `tts.say("Input Text Here")`
<a name = "connecting">
# Building a DC motor<a name = "building-a-dc-motor">
## Part I - Building a brushed DC motor<a name = "part-i">
The aim of this part is to create a DC motor. For this we have:
* Comakingape
* Pa-talkper Clips
* 10m Copper Wire
* 2 nails

### Step 1 - Building the commutator<a name = "i-step-1">
The commutator is the moving part of the motor that will rotate within the magnetic field. This was started by placing copper
tape on opposite sides of the cork, ensuring to leave a gab between each strip. This is needed as the tape will later be connected
to two ends of the copper wire, and will be used to reverse the current.

_The image below shows the commutator with 1 long strip of copper tape rather than 2 pieces of tape before it was created_
![Step 1 <a name = "building a DC motor">Image][Image1]
<a name = "part-I">
[Image1]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_092122.jpg "Commutator with copper tape"

### Step 2 - Support Shaft<a name = "i-step-2">
A support shaft for the commutator was made using two nails placed into the two ends of the cork. This will allow the commutator
to move freely when placed on a support.
![Step 2 Image 1][Image2]
<a name = "step-1">
[Image2i-]: https://github.com/NodrogJRB/ROCO222/commit/867ef6f2d274338247e842fc8fdd3f470fddd7d8

Tape was then added around the copper tape to better hold it in place:
![Step 2 Image 2][Image 3]

[Image3]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_093243.jpg

### Step 3 - Winding the armature coil<a name = "i-step-3">
This was the step that took the longest! A coil was wraped lengthways around the cork forming a coil.
It was recomended<a name = "I-step-2"> for this that the coil had 60 turns minimum and we managed aroudn 139 turns (give or take 2 or 3 due to the
monotonus of counting the turns whilst wrapping).
![Step 3 Image 1][Image4]

Both ends of the wire were left free at the top (end with copper tape), then each end was soldered onto one of the two pieces of
copper tape. This is how the current will flow through the coil.
Due to the enamal on the copper wire preventing elctricity from conduction through it, sandpaper was needed to sand it off
the ends before soldering.

The resistance of the coild was measured at =~7 ohms.
![Step 3 Image 2][Image5]

[Image4]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_100120.jpg
[Image5]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_101433.jpg

### Step 4 - Building the shaft support & magnet brackets<a name = "i-step-4">
The shaft support was made by using two paper clips, and stretching out one end, and curling it round into a small loop as shown
in the image. This will later create an ideal support for the twon nails used as a support shaft.

The magnet bracket was made by simply bending the paper clip into an L-shape.
![Step 4 Image][Image6]

[Image6]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_101714.jpg

### Step 5 - Assembly<a name = "i-step-5">
Firstly the wooden base needed to be evenly devided horizontally and vertically (which we didn't need to do as it was already
done). This allowed the paper clips to be equally fastened.

The two support paper clips were screwed into the board lengthways along the board. The commutator could then be rested between
these.
The two magnet brackets were attached width ways, at a 90° to the commutator. Two of each of the magnets were then put onto the
brackets, which could support themselves due to their magnetic strength. These were attached so that all the magnets were in the
same direction and attracting to one another.
![Step 5 Image][Image7]

[Image7]: https://github.com/NodrogJRB/ROCO222/blob/master/Images/20171002_102626.jpg
