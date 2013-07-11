Warning
=======

This is a Work In Progress!  There are no warrantees or Guarantees of any type that this code is useable for anything.  

But I hope it is.

This simple sketch is my test sketch for testing out the servos on Trossen Robotics PhantomX hex and quad robots.

Installation
============

This sketch uses the Serial port to output information to be displayed and to receive simple text commands. 

I normally use it, with the Arduinos IDE Serial monitor.  The program is configured to output at the baud rate
of 38400.

If you use an FTDI cable to program your robot, you can simply use this same serial port.  If you are like me and
use an ISP to program your robot, you can either disconnect the XBee and connect the FTDI serial connection or do
what I normally do, which is to remove the XBee from the Arbotix Commander and put it into the USB XBee adapter and
plug that into your PC and use that serial port to talk to your robot.   Side note: I actually cheat ans simply have
a spair XBee configured the same as the Commanders XBee, so I don't have to swap.

There is a #define at the start of the program: #define QUAD_MODE
that I most often have commented out.  When it is defined it only talks to the servos that are defined for the standard
PhantomX quad, otherwise it will try to input and output to the standard 18 servos of the hexapod.

Commands
========

The program takes real simple text strings as commands.  I will  describe a few of the ones I use most often.

0<cr> - Free all servos
-----------------------
Frees all of the servos, such that they no longer try to hold a position.

1<cr> - All servos center
-------------------------
This tells all of the servos to go their center position.  I use this to help verify that the robot is constructed
properly.  For the Mark 2 version of the PhantomX robots, all of the Coxa(Hip) servos should be pointing straight 
out from their connection and the Femur(Knee) andTibia(Ankle) servos should be parallel to the ground.  

2 [<servo>] <position> [<speed>]<cr> - Moves a servo
----------------------------------------------------
I often use this to help me to verify a servo is working properly.  Examples include:
    2 2 450<cr> - Move servo 2 to value 450 - Only one servo should move
    2 1 450<cr> - Likewise for servo 1.  Note: if servos have reset multiple might move, which is an error!
    
4<cr> - Get Servo positions.
----------------------------
When I suspect that maybe a servo id has been reset, I will use this command to print out the current position
of all of the servos.  It should print out a valid value for each of the servos.  If it fails to get a valid
value it will print something like -1 and print out a retry line.  If you get this, often #1 will error out as
well as one or more others.  In this case I normally find the other ones probably reset their id to #1.  I often
verify this by using the command 2 above.  First I will try with the ids other than #1 that failed, probably nothing
moves.  I then try with #1 and multiple servos may move.

8 <old id> <new id> <cr> - Set Servo id
------------------------
If I found that one or more servos has reset it's id to #1, I use this command to reset them.  Note: it will 
potentially change all of the servos with the Old id to the New id, so you want to unplug the real #1.  Example
    8 1 8<cr> - will change servo 1 to servo 8
Note: if a servo that changed ids is connected to the real #1 like #5, you can do this with multiple steps, something like:
First unplug other servos from the real #1, then give the real #1 a different id, like 42 (8 1 42 <cr>), 
then plug the other servos back in and change the one back to its real id (8 1 5 <cr>), and then change the real #1 back
to 1 (8 42 1)

9 <id><cr> - Print information out about a servo.
-------------------------------------------------
Prints out more information out about a specific servo.

t<cr> - Toggle track servos
---------------------------
Turns on tracking of servos.  You typically want to free one or all of the servos.  You can then manually move the 
servos and it will print out the servo position as it moves.  It gives it both in servo position as well as angle in
tenths of a degree.  When you turn off the tracking, it will print out the minimum and maximum values it saw for all 
of the servos while they were being tracked.

h [<id>]<cr> - Hold current position
------------------------------------
Tells the servo to hold it's current position.  If no servo number is given it holds all of them.

f [<id>]<cr> - Free Servo
-------------------------
Allows the servo to go free.  If no id is given it is like command 0


Again Warning
=============

This is a WIP - No promises or guarantees!
