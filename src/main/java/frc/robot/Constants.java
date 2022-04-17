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
 */
public final class Constants {
    // Climber needs to be updated
    public final static class ClimberConstants {
        // CAN Bus
        public static final int kMasterMotorID = 10;
        public static final int kFollowerMotorID = 0;

        // Motor direction
        public static final boolean kIsMasterInverted = true;

        // Speeds to raise (extend) and lower (climb) mechanism
        public static final double kExtensionSpeed = 0.4;
        public static final double kClimbingSpeed = -0.4;

        // Positional limits
        public static final double kExtendLimit = 230000;
        public static final double kRetractLimit = 1000; // lower if climber is drifting upwards

        // PID Loop for quick climbing
        // kP Formula: (throttle% * 1023) / error (ex 4096 = 2 revs)
        public static final double kF = 0;
        public static final double kP = 0.0999; 
        public static final double kI = 0;
        public static final double kD = 0;

        // Other variables to set CTRE config
        public static final int kSlotIdx = 0;
        public static final int kTimeoutMs = 30;
    }

    public final static class ControllerConstants {
        // Corresponds to port controller is plugged into on the DS
        public static final int kControllerPort = 0;
    }

    public final static class DriveConstants {
        // CANBus
        public static final int kFrontLeftMotorID = 3;
        public static final int kBackLeftMotorID = 4;
        public static final int kFrontRightMotorID = 1;
        public static final int kBackRightMotorID = 2;

        // Directional Logic
        public static final boolean kIsLeftSideInverted = false;
        public static final boolean kIsRightSideInverted = true;
        public static final boolean kGyroReversed = true;
        public static final boolean kIsRotReversed = false;

        // Physical Robot Properties (in inches)
        public static final double kTrackWidth = 22;
        public static final double kWheelRadius = 3;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Ramp!
        public static final double kRampInSec = 0.1875;

        // Characterization Constants
        public static final double kP = 3.3619; // might tweak this
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.59217;
        public static final double kV = 2.4309;
        public static final double kA = 0.37474;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Max Speeds
        public static final double kVelocityMax = 0.9144 ; // 6ft/s
        public static final double kAccelerationMax = 0.6096; // 4ft/s^2
    }

    public final static class IndexerConstants {
        // CAN
        public final static int kIndexerMotorID = 8;

        // Speed
        public final static double kIndexerSpeed = 0.4;

        // Direction
        public final static boolean kIndexerInverted = false;
    }

    public final static class LimelightConstants {
        // Targeting speeds and constants
        public final static double kP = 0.01;
        public final static double kMinMovementSpeed = 0.27; // So robot can move a smidge.
        public final static double kAngleThreshold = 0.5; // can lower to make tolerance tighter
        public final static int kDefaultPipeline = 9; // ideally this is an empty pipeline
        public final static int kRedPipeline = 0;
        public final static int kBluePipeline = 1;
    }

    public final static class PickupConstants {
        public final static int kPickupMotorID = 7;
        public final static boolean kPickupInverted = false;
        public final static double kPickupSpeed = 0.8; // Was 0.6 at some point
    }

    public final static class PivotConstants {
        // CAN
        public final static int kPivotMotorID = 9;

        // Required for CTRE config
        public final static int kSlotIdx = 0;
        public final static int kTimeoutMs = 30;

        // Constants for PID control
        public final static double kF = 0;
        public final static double kP = 0.04995;
        public final static double kI = 0;
        public final static double kD = 0;
        
        // Positions for raising and lowering the pivot
        // TURN ROBOT ON WHEN PIVOT IS UP!!!
        public final static double kRaisedPose = -2000;
        public final static double kLoweredPose = -90000;

        // Speed at which to drive the pivot at (unused)
        public final static double kSpeed = 0.2;
    }

    public final static class ShooterConstants {
        // CAN IDs
        public final static int kMasterMotorID = 5;
        public final static int kFollowerMotorID = 6;

        // Characterization Values
        public final static double kS = 0.65743;
        public final static double kV = 0.53916;
        public final static double kA = 0.021606;
        public final static double kP = 0.4432;
        public final static double kI = 0;
        public final static double kD = 0;

        // Other properties of the shooter that we need
        public final static double kEncoderResolution = 2048;
        public final static double kGearRatio = 1.5; // 3:2
        public final static double kWheelDiameter = 3.75; // is in inches

        // These are arbritrary numbers, but they work.
        // Be very, very careful with the manual speed setter. The threshold
        // doesn't move so it's stuck to whatever it is set to here.
        public final static double kHighGoalSpeed = 10.4; //10.4, 8, 
        public final static double kHighGoalThresh = 8; // need to tweak as well, 8
        public final static double kLowGoalSpeed = 10; // Need to tweak
        public final static double kLowGoalThresh = 8;

        // Set inversion within subsystem. Default rot. is CCW.
        // Back motor will always be opposite this.
        public final static boolean kFrontShooterMotorInverted = true; // left invert
    }

    // These actually aren't test constants, they're used for the custom speed settings.
    public final static class TestConstants {
        public final static double kSpeedIncrement = .01;
        public final static double kCustomSpeedIncrement = 0.1; // for the shooter
    }

    // Put pathnames for trajectories here, all will be loaded at robot initialization
    public final static class TrajectoryPathnames {
        // For JsonTrajRoutine
        public static final String kJsonTrajPath = "StraightBack.wpilib.json";

        // kBackupTraj is a common backup trajectory, while center and far belong to
        // each of their respective routines
        public static final String kBackupTraj = "BackOutTest.wpilib.json";
        public static final String kCenterBallTraj = "ABCD.wpilib.json";
        // public static final String kCenterBallTraj = "CONFIRMED_CENTER.wpilib.json";
        public static final String kFarBallTraj = "CONFIRMED_FAR.wpilib.json";
    }
}
