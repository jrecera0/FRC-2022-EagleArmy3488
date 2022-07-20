/**
 * Contains all the commands that can be used to control the robot and its functions, including driving
 * and running all subsystems such as indexing and shooting. Note about the commands, all of the print
 * statements have been commented out, they're okay to have but we had to disable them during competition
 * because of the performance drop we were experience on the Driver Station just to see if it would help.
 * 
 * <p> For the record, the issue was actually having ShuffleBoard running in the background, for some
 * reason it was heavily using the Driver Station's CPU and was leading to a lot of latency between the
 * laptop and FMS so be careful when using it. Likewise, print statements can also cause issues so be
 * careful when printing a ton of data too.
 * 
 * <p> Please note that I will NOT be going over the Command-based programming paradigm here in documentation,
 * FRC has some pretty extensive docs that cover this. However, I will be leaving small notes on a few methods
 * just to denote why certain things are set the way they are.
 * 
 * <p> Also note that I'm 99% sure I did the programming for the Command-based structure super inefficiently
 * and that there's a far easier way to have basic controls be implemented. A lot of these are mostly just
 * toggles to move a subsystem one direction or another, or to set specific speeds or modes. I have a feeling
 * this could possibly be fixed via suppliers, whether they be in the form of doubles or booleans or at least
 * have sort of state management system so that we don't end up with 26 different commands that all
 * accomplish similar things, but it's definitely something to keep in mind for the future.
 */
package frc.robot.commands;
