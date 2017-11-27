Table of Contents
=================
* [Part I - My lab journal](#part-i)
  1. [Write your first Markdown document](#i-step-1)
  2. [Command-line 101](#i-step-2)
  3. [Create a GitHub account](#i-step-3)
  4. [Your first git repository](#i-step-4)
  5. [Your first commit](#i-step-5)
  6. [Version Tracking](#i-step-6)
  7. [Going social](#i-step-7)
* [Part II - Hack into a robot](#part-ii)

Practical 1: Software engineering for robotics
==============================================
Part I - My lab journal<a name = "part-i">
------------------------------------------
### Step 1 - What is markdown<a name = i-step-1>
Markdown is basically a way to create a text document, then edit and styalize it using syntax for display on the web.
For example using '*Hello*' in the text editor will look like *Hello* on the web.

This allows for the text to be edited by the created in any way desired without having to worry about selecting fonts or adding titles.

* `*Italic*` or `_Italic_` = *Italic*
* `**Bold**` or `__Bold__` = **Bold**
* `#Header` will create a header like the top of the page
* `##Header` will create a header like the top of this list
* `###Header` will make a third level header, and so on.
* `>This symbol can be used to make quotes` >This symbol can be used to make quotes
* Using the ``` ` ``` Symbol allows for *Italic* to be shown as `*Italic*` (``` `*Italic*` ```)
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
  
### Step 2 - Using the Terminal<a name = i-step-2>
* `ls` shows the availabale folders within the currently selected directory.
* `cd /tmp` this takes the user to the temporary directory of '/tmp'
* `cd $HOME` takes us aback to the Home directory. Using the $ at the start uses the HOME as a representative of something else, 
in this case '/home/student'
* `mkdir` is used to make directories
* `echo "Hello"` will display Hello in the terminal
* `echo "Hello" > hello.md` produces a '.md' file with the text 'Hello'
* `cat hello.md` displays the text within the file 'hello.md'
* `cp hello.md hello-again.md` copies the file 'hello.md' and renames it to 'hello-again.md'
* `mv hello-again.md hello-hello.md` moves and renames the file.
* `rm hello.md` removes the file
* `rm -rf`
* `cat /proc/cpuinfo` displays the text contents of the filhacking-the-robot'

### Step 3 - Create a GitHub account<a name = i-step-3>
Creating a github account is incredibly usefull for managing git files and to access them from anywhere.

### Step 4 - Your first git repository<a name = i-step-4>
Git is used to track the changes within a particular file. Firstly the '.git/' file needs to be created using 'git init'. From here on 
this can be used to log the changes within a given file.

Once git has been initialised, a name and email needs to be set using 'git config user.name "Firstname Surname"' and 
'git config user.email "email"
So in my case it'd be 'git config user.name "Ben Gordon' and 'git config user.name `ben.gordon@students.plymouth.ac.uk`'

### Step 5 - Your first commit<a name = i-step-5>
Then the journal file can be added to the git folder using 'git add journal.md', then create a new commit using 'git commit'. 
When doing this a short (usually less than 72 characters) summary of the changes should be typed on what has been added/changed to 
the file.

### Step 6 - Version Tracking<a name = i-step-6>
Version tracking can be used to view the previous updates to the file. I can then roll back to any previous version if anything goes 
wrong or gets corrupted.

### Step 7 - Going social<a name = i-step-7>
Git hub can technically be used for any number of people to work on a single project. The files can be pulled from git hub
onto your own system to work on, and pushed back to git hub, therefore updating the project.

Part II - Hack into a robot<a name = "part-ii">
-----------------------------------------------
After browsing round many websites, we finally figured out how to hack the robot and make it talk!

#### Connecting to chapman<a name = "connecting-to-chapman">
##### Finding the IP<a name = "finding-the-ip">
Firstly we had to connect to the chapman robot.

To do this we first went to chapman.local in the web browser. This brought us to a configuration page for chapman.

From this we could find the robot's IP address by going to the **Network Settings** and clicking on **Wired**. This showed us that the IP was **192.167.0.184**.

##### Connecting<a name = "connecting">
Once we had the IP, we could then connect to chapman through the terminal by typing `ssh nao@192.167.0.184`. This connects us to the robot to be able to navigate files and use python to control the speech.

#### Making Chapman talk<a name = "making-chapman-talk">
To make him talk was a simple task. first we typed `py<a name = "finding-the-ip">thon` into the console, which opened a python console to input code.

Then the following commands were entered:
* `from naoqi import ALProxy`
* `tts = ALProxy("ALTextToSpeech", "localhost", 9559)` - this enables the text to speach and assigns
.. it `tts`
* finally Chapman can be made to say anything by inputing `tts.say("Input Text Here")`
