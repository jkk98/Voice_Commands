Voice_Commands
==============

Voice Commands using ROS on Turtlebot

---About---
This Project allows for Turtlebot to hear, repeat, and implement commands.

---Setup---
Need to install PocketSphinx at http://wiki.ros.org/pocketsphinx. Just follow the installation.
Also need to install sound_play at http://wiki.ros.org/sound_play.
	Get the package from the github source and then run the following commands for setting it up:
	$ rosdep install sound_play
	$ rosmake sound_play
Might need to set the correct message types in the package.xml file
Go to http://www.speech.cs.cmu.edu/tools/lmtool.html for creating a new dictionary

---Launch Files---
voice_cmd_test.launch has the most up to date implementation.
voice_cmd.launch is just the original voice command launch file.
voice_tests.launch is for testing with the test dictionaries with more words. (I need to use better wording for these titles)
test_bouncer.launch is used for testing the launch of new nodes during runtime.


---Scripts---
Mostly just the things covering the launch files
Has my current attempt at getting the parameterization node to do anything.

---Dict---
Need to clean up this area and have a decent dictionary and language model that covers all the areas wihtout having too many words
Maybe add multiple language models or even dictionaries for parameterized commands.

---World---
Has a bunch of different world files we can work with

