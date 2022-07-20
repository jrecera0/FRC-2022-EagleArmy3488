// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p> From this point onwards, additional information is being provided by James
 * 
 * <p> The Constants class is set up in such a way where we create a series of subclasses to group
 * variables with their subsystems in a more organized manner. This tends to work out better than just
 * putting all the constants into one class, but at the end of the day, you are free to organize this
 * however you please.
 * 
 * <p> ID refers to the motor CAN ID number set in Phoenix Tuner. Speed is based on a percentage of
 * the total power supplied to the motor itself, with 1.0 representing 100%. Limits are based on raw
 * encoder units, 2048 representing one full rotation of the shaft. kF, kP, kI, kD are constants for
 * PID tuning, you can generate these in phoenix tuner or tweak by hand.
 * 
 * <p> For long-term purposes, I highly recommend creating an even more streamlined way of setting uo
 * all your constants and where they go. For example, these subclasses are alphabetized A-Z, but all the
 * actual constants are in a weird mish-mash order for each section. Might be worth streamlining the
 * constants to be alphabetized too, either that or come up with an order of, "CAN, speeds, PID constants"
 * that you can apply to each section.
 * 
 * <p> Default motor rotation if you are staring down the shaft is CCW.
 */
public final class Constants {
    /**
     * Contains all the constants for the Climber subsystem. For this code specifically, the limits were
     * sorta poorly implemented under the assumption that when robot code started, they were in the LOWERED
     * positions. This is important, because if the robot is powered on while the climbers are raised, the
     * lower limit will be at that RAISED position, and the upper limit will be physically impossible to
     * reach. Also the climber will sorta just be stuck. Same thing should be noted for the pivot. Make sure
     * it is raised and INSIDE the frame perimeter with the pickup all the way in. Otherwise, you can and
     * will smash the entire pickup into the ground.
     * 
     * <p> please implement this better in the future, potentially a sort of calibration mode where you can
     * reset the neutral position.
     * 
     * <p> kP Formula: (throttle% * 1023) / error (ex 4096 = 2 revs)
     */
    public final static class ClimberConstants {
        /**
         * CAN ID for the climber master motor.
         **/
        public static final int kMasterMotorID = 10;

        /**
         * CAN ID for the climber follower motor.
         */
        public static final int kFollowerMotorID = 0;

        /**
         * Specifies whether or not the master motor moves CCW or CW.
         */
        public static final boolean kIsMasterInverted = true;

        /**
         * Speed to raise (extending) the climber mechanism at.
         */
        public static final double kExtensionSpeed = 0.4;

        /**
         * Speed to lower (climbing) the climber mechanism at.
         */
        public static final double kClimbingSpeed = -0.4;

        /**
         * Upper limit of the climber's position.
         */
        public static final double kExtendLimit = 230000;

        /**
         * Lower limit of the climber's position. Might have to lower if the climber starts to drift.
         * 
         * <p> For the future, might wanna try and figure out a way to better set these limits to
         * prevent drifting!
         */
        public static final double kRetractLimit = 1000;
        
        /**
         * Feedforward constant for climbing, don't know if this does anything for the climber.
         */
        public static final double kF = 0;

        /**
         * P constant for PID control loop.
         */
        public static final double kP = 0.0999; 

        /**
         * I constant for PID control loop.
         */
        public static final double kI = 0;

        /**
         * D constant for PID control loop.
         */
        public static final double kD = 0;

        /**
         * Config slot index to pick on the motor
         */
        public static final int kSlotIdx = 0;

        /**
         * Configuration timeout period
         */
        public static final int kTimeoutMs = 30;
    }

    /**
     * Contains all the constants for the Xbox controller.
     */
    public final static class ControllerConstants {
        /**
         * Controller port number the controller is plugged into.
         */
        public static final int kControllerPort = 0;
    }

    /**
     * Contains all the constants for the robot drive train. 
     * 
     * <p> For 2022, we used a tank drive assemblycwith having one master motor on each side,
     * and a follower motor that would more or less follow. This is different because in
     * previous years, we would have actually used a SpeedControllerGroup in order to group
     * our motors together. This has been deprecated, so instead we opted to use CTRE's library
     * to set up a master-slave configuration for the motors and went from there.
     * 
     * <p> If reusing this file or constants, PLEASE make sure to recharacterize and to change
     * out the constants. More specifically, kS, kV, kA, and kP will definitely change with a
     * change in the mass distribution and weight of the robot so it is important that you
     * recharacterize whenever there's a substantial change in weight.
     */
    public final static class DriveConstants {
        /**
         * Front-left drive train motor CAN ID number.
         */
        public static final int kFrontLeftMotorID = 3;

        /**
         * Back-Left drive train motor CAN ID number.
         */
        public static final int kBackLeftMotorID = 4;

        /**
         * Front-right drive train motor CAN ID number.
         */
        public static final int kFrontRightMotorID = 1;

        /**
         * Back-right drive train motor CAN ID number.
         */
        public static final int kBackRightMotorID = 2;

        /**
         * Set true if the left side of the drive train is reversed,
         */
        public static final boolean kIsLeftSideInverted = false;

        /**
         * Set true if the right side of the drive train is reversed.
         */
        public static final boolean kIsRightSideInverted = true;

        /**
         * Set the gyro as reversed or not. If using the NavX, if it is facing up, then it should
         * be reversed. However, if the gyro is upside down, then the gyron should NOT be reversed.
         * This is because of how trajectories are handled, versus how the gyro reports the robot's
         * heading.
         */
        public static final boolean kGyroReversed = true;

        /**
         * I... I don't know what this is, I don't think this is even used. Why does this exist? Why
         * do any of exist? Why do I do this at 2am at night? I know not the answer to any of these questions.
         */
        public static final boolean kIsRotReversed = false;

        /**
         * Distance between the tracks of the drive train in inches.
         */
        public static final double kTrackWidth = 22;

        /**
         * Radius of the drive train wheels in inches.
         */
        public static final double kWheelRadius = 3;

        /**
         * Gear ratio of the gearboxes installed in the drivetrain.
         */
        public static final double kGearRatio = 10.71;

        /**
         * Resolution of the built-in encoders of the Falcon-500s. In theory, you should be able to change this
         * number out in the future and the math might still be fine for robot odometry if we ever have motors
         * with a different resolution.
         */
        public static final double kEncoderResolution = 2048;

        /**
         * Set the period of time to have the robot ramp up to the desired speed, in seconds.
         */
        public static final double kRampInSec = 0.1875;

        /**
         * P constant of the PID control loop for the drive train
         */
        public static final double kP = 3.3619;

        /**
         * I constant of the PID control loop for the drive train.
         */
        public static final double kI = 0.0;

        /**
         * D constant of the PID control loop for the drive train.
         */
        public static final double kD = 0.0;

        /**
         * S constant of the feedforward control loop for the drive train.
         */
        public static final double kS = 0.59217;

        /**
         * V constant of the feedforward control loop for the drive train.
         */
        public static final double kV = 2.4309;

        /**
         * A constant of the feedforward control loop for the drive train.
         */
        public static final double kA = 0.37474;

        /**
         * RamseteB constant given from FRC docs for trajectories.
         */
        public static final double kRamseteB = 2;

        /**
         * RamseteZeta constant given from FRC docs for trajectories.
         */
        public static final double kRamseteZeta = 0.7;

        /**
         * (Might not even be used) Maximum velocity of the drive train in m/s
         */
        public static final double kVelocityMax = 0.9144 ; // 6ft/s

        /**
         * (Might not even be used) Maximum acceleration of the drive train in m/s
         */
        public static final double kAccelerationMax = 0.6096; // 4ft/s^2
    }

    /**
     * Contains all the constants for the Indexer subsystem.
     */
    public final static class IndexerConstants {
        /**
         * CAN ID number of the indexer
         */
        public final static int kIndexerMotorID = 8;

        /**
         * Spped to run the indexer at
         */
        public final static double kIndexerSpeed = 0.4;

        /**
         * Specify if the indexer motor needs to be reversed or not
         */
        public final static boolean kIndexerInverted = false;
    }

    /**
     * Contains all the constants for the Limelight.
     * 
     * <p> All of these are likely prone to changing, so you're free to remove
     * or add to these as needed. Be sure to reference limelight docs for more
     * info on numbers and data!
     */
    public final static class LimelightConstants {
        /**
         * P constant for PID control for pivoting the robot towards balls on the field.
         */
        public final static double kP = 0.01;

        /**
         * Minimum speed needed to have the robot move
         */
        public final static double kMinMovementSpeed = 0.27; // So robot can move a smidge.

        /**
         * Tolerance for how close the robot needs to be within the center angle to be considered centered.
         * Can raise or lower to make tolerance tighter or looser, highly recommend tweaking.
         */
        public final static double kAngleThreshold = 0.5;

        /**
         * Empty pipeline to switch to when not targetting. Not used in code as far as I can tell.
         */
        public final static int kDefaultPipeline = 9; // ideally this is an empty pipeline

        /**
         * Pipeline used to target red power cells on the field
         */
        public final static int kRedPipeline = 0;

        /**
         * Pipeline used to target blue power cells on the field
         */
        public final static int kBluePipeline = 1;
    }

    /**
     * Contains all the constants for the Pickup subsystem.
     */
    public final static class PickupConstants {
        /**
         * CAN ID number of the pickup motor
         */
        public final static int kPickupMotorID = 7;

        /**
         * Specify if the motor of the pickup should be inverted or not
         */
        public final static boolean kPickupInverted = false;

        /**
         * Speed pickup should be run at
         */
        public final static double kPickupSpeed = 0.8; // Was 0.6 at some point
    }

    /**
     * Contains all the constants for the pivot subsystem that raises the pickup.
     */
    public final static class PivotConstants {
        /**
         * CAN ID of the motor for the pivot
         */
        public final static int kPivotMotorID = 9;

        /**
         * Index number of the config slot to select on the motor
         */
        public final static int kSlotIdx = 0;

        /**
         * Timeout configuration period
         */
        public final static int kTimeoutMs = 30;

        /**
         * Feedfoward gain thingy, it's 0.
         */
        public final static double kF = 0;

        /**
         * P constant of pivot for PID control loop
         */
        public final static double kP = 0.04995;

        /**
         * I constant of pivot for PID control loop
         */
        public final static double kI = 0;

        /**
         * D constant of pivot for PID control loop
         */
        public final static double kD = 0;
        
        /**
         * Upper limit for pivot being raised. Ensure default position is RAISED.
         */
        public final static double kRaisedPose = -2000;

        /**
         * Lower limit for pivot being lowered to the front of the robot.
         */
        public final static double kLoweredPose = -90000;

        /**
         * Speed to run the pivot mechanism at. This might not even be used.
         */
        public final static double kSpeed = 0.2;
    }

    /**
     * Contains all the constants for the Shooter subsystem.
     */
    public final static class ShooterConstants {
        /**
         * CAN ID number for the master motor of the shooter
         */
        public final static int kMasterMotorID = 5;

        /**
         * CAN ID number for the follower motor of the shooter
         */
        public final static int kFollowerMotorID = 6;

        /**
         * S constant for shooter feedforward loop
         */
        public final static double kS = 0.65743;

        /**
         * V constnat for shooter feedforward loop
         */
        public final static double kV = 0.53916;

        /**
         * A constant for shooter feedforward loop
         */
        public final static double kA = 0.021606;

        /**
         * P constant for shooter PID control loop
         */
        public final static double kP = 0.4432;

        /**
         * I constant for shooter PID control loop
         */
        public final static double kI = 0;

        /**
         * D constant for shooter PID control loop
         */
        public final static double kD = 0;

        /**
         * Encoder resolution of the Falcon 500s
         */
        public final static double kEncoderResolution = 2048;

        /**
         * Gear Ratio of the shooter as a decimal (3:2)
         */
        public final static double kGearRatio = 1.5; // 3:2

        /**
         * Physical diameter of the shooter wheels in inches
         */
        public final static double kWheelDiameter = 3.75; // is in inches

        // These are arbritrary numbers, but they work.
        // Be very, very careful with the manual speed setter. The threshold
        // doesn't move so it's stuck to whatever it is set to here.
        /**
         * (Warning: arbritrary number) Speed for shooting to the high goal in "m/s". Note
         * on the shooter itself, this is very janky. Supposedly the characterization and math
         * that I did should in theory be in meter's per second but the actual speed that's output
         * when we use the PID control loop ends up looking... not that way. They're numbers, that's
         * for sure.
         */
        public final static double kHighGoalSpeed = 10.4; //10.4, 8, 

        /**
         * Threshold speed for when the indexer should start moving the balls to the shooter. Once
         * again, this is in "m/s" aka not actually m/s.
         */
        public final static double kHighGoalThresh = 8; // need to tweak as well, 8

        /**
         * This was a theoretical speed for the lower goal but it ended up not being used. Ignore
         * this number.
         */
        public final static double kLowGoalSpeed = 10; // Need to tweak

        /**
         * Another low goal theoretical speed but as a threshold, this is basically unused?
         */
        public final static double kLowGoalThresh = 8;

        /**
         * Set whether or not the front shooter motor is inverted or not. The back motor
         * will always be opposite the direction of whatever is specified here.
         */
        public final static boolean kFrontShooterMotorInverted = true; // left invert
    }

    /**
     * Despite the name, this contains the constants for the CustomSpeedController class
     * to increment or decrement the speeds on the fly.
     */
    public final static class TestConstants {
        /**
         * This is the amount to increase and decrease subsystem speeds by when using the
         * custom speed management command.
         */
        public final static double kSpeedIncrement = .01;

        /**
         * This is a special amount to increase and decrease the speed by specfically for
         * the shooter when using the custom speed management command.
         */
        public final static double kCustomSpeedIncrement = 0.1; // for the shooter
    }

    /**
     * Contains each of the trajectory pathnames that correspond to each autonomous routine.
     */
    public final static class TrajectoryPathnames {
        /**
         * Pathname to the trajectory to run with JsonAutoRoutine.
         */
        public static final String kJsonTrajPath = "StraightBack.wpilib.json";

        // kBackupTraj is a common backup trajectory, while center and far belong to
        // each of their respective routines
        /**
         * Pathname to the common backup trajectory used in all the official competition paths that pulls
         * the robot away from the tarmac.
         */
        public static final String kBackupTraj = "BackOutTest.wpilib.json";

        /**
         * Pathname to the trajectory to run with CenterBallRoutine.
         */
        public static final String kCenterBallTraj = "ABCD.wpilib.json";
        // public static final String kCenterBallTraj = "CONFIRMED_CENTER.wpilib.json";

        /**
         * Pathname to the trajectory to run with FarBallRoutine.
         */
        public static final String kFarBallTraj = "CONFIRMED_FAR.wpilib.json";
    }
}
