# What is Markdown?
Markdown is basically a way to create a text document, then edit and styalize it using syntax for display on the web.
For example using '*Hello*' in the text editor will look like *Hello* on the web.

This allows for the text to be edited by the created in any way desired without having to worry about selecting fonts or adding titles.

## Key Syntax
* '*Italic*' or '_Italic_' = *Italic*
* '**Bold**' or '__Bold__' = **Bold**
* '#Header' will create a header like the top of the page
* '##Header' will create a header like the top of this list
* '###Header' will make a third level header, and so on.
* '>This symbol can be used to make quotes' >This symbol can be used to make quotes
* Using the ''' Symbol allows for *Italic* to be shown as '*Italic*' (''*Italic*'')
* Create lists using '*', '1.' or '-':

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
* 'ls' shows the availabale folders within the currently selected directory.
* 'cd /tmp' this takes the user to the temporary directory of '/tmp'
* 'cd $HOME' takes us aback to the Home directory. Using the $ at the start uses the HOME as a representative of something else, in this case '/home/student'
* 'mkdir' is used to make directories
* 'echo "Hello"' will display Hello in the terminal
* 'echo "Hello" > hello.md' produces a '.md' file with the text 'Hello'
* 'cat hello.md' displays the text within the file 'hello.md'
* 'cp hello.md hello-again.md' copies the file 'hello.md' and renames it to 'hello-again.md'
* 'mv hello-again.md hello-hello.md' moves and renames the file.
* 'rm hello.md' removes the file
* 'rm -rf'
* 'cat /proc/cpuinfo' displays the text contents of the file 'cpuinfo'

# Using Git
Git is used to track the changes within a particular file. Firstly the '.git/' file needs to be created using 'git init'. From here on this can be used to log the changes within a given file.

Once git has been initialised, a name and email needs to be set using 'git config user.name "Firstname Surname"' and 'git config user.email "email"
So in my case it'd be 'git config user.name "Ben Gordon' and 'git config user.name "ben.gordon@students.plymouth.ac.uk'

Then the journal file can be added to the git folder using 'git add journal.md', then create a new commit using 'git commit'. When doing this a short (usually less than 72 characters) summary of the changes should be typed on what has been added/changed to the file.
