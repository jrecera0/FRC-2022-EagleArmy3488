// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoroutines;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryPathnames;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.DropPivotCmd;
import frc.robot.commands.FastPickupCmd;
import frc.robot.commands.PickupCmd;
import frc.robot.commands.RaisePivotCmd;
import frc.robot.commands.ResetDTPoseCmd;
import frc.robot.management.TrajectoryLoader;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/**
 * This class is where we set up and create all of the {@code FarBallRoutine}. In theory,
 * you could very well just create the {@link SequentialCommandGroup} within the
 * {@link frc.robot.RobotContainer} class but for better understanding and organizational purposes,
 * I made the autoroutines as separate classes instead. The idea here is that you can
 * just look at the autoroutine as a whole and understand what it does by looking at a single class,
 * and modify it to your needs if the requirements change, without having to go digging in
 * {@link frc.robot.RobotContainer} where a number of other things are already happening.
 * 
 * TODO: Add diagram!
 */
public class FarBallRoutine {
    private Trajectory backupTrajectory;
    private Trajectory pickupTrajectory;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Indexer indexer;
    private Pickup pickup;
    private Pivot pivot;
    
    /**
     * Use this to set up the {@code FarBallRoutine} with all required subsystems and the
     * {@link TrajectoryLoader} to load the correct sequence of trajectories as specified in
     * the {@link frc.robot.Constants} class.
     * @param trajectoryLoader Trajectory loader to grab the trajectories
     * @param driveTrain {@code DriveTrain} to be used for driving the trajectories
     * @param shooter {@code Shooter} to be used for shooting power cells
     * @param indexer {@code Indexer} to be used to queue power cells towards the {@code Shooter}
     * @param pickup {@code Pickup} to be used to pick up power cells from the field
     * @param pivot {@code Pivot} to be used to raise and lower the pickup throughout the routine
     */
    public FarBallRoutine(TrajectoryLoader trajectoryLoader, DriveTrain driveTrain, Shooter shooter, Indexer indexer, Pickup pickup, Pivot pivot) {
        backupTrajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kBackupTraj);
        pickupTrajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kFarBallTraj);
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.pivot = pivot;
        this.pickup = pickup;
    }

    /**
     * Returns the assembled {@link SequentialCommandGroup} with the entire autonomous routine.
     * This is what you realistically want to be editing when you're making changes to the routine
     * as a whole, outside of changing trajectories.
     * @return {@code FarBallRoutine} {@link Command} 
     */
    public Command getCommand() {
        return new SequentialCommandGroup(
            new AutoShootCmd(indexer, shooter, pickup).withTimeout(1.5),
            new RamseteCommand(
                backupTrajectory.transformBy(driveTrain.getPose().minus(backupTrajectory.getInitialPose())), 
                driveTrain::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                driveTrain.getSimpleMotorFeedForward(),
                driveTrain.getKinematics(),
                driveTrain::getDriveWheelSpeeds,
                driveTrain.getLeftPIDController(), 
                driveTrain.getRightPIDController(),
                driveTrain::setTankDriveVolts,
                driveTrain
            ),
            new ResetDTPoseCmd(driveTrain),
            new ParallelCommandGroup(
                new DropPivotCmd(pivot).withTimeout(1).andThen(new FastPickupCmd(pickup).withTimeout(4).andThen(new RaisePivotCmd(pivot).withTimeout(1))),
                new RamseteCommand(
                    pickupTrajectory.transformBy(driveTrain.getPose().minus(pickupTrajectory.getInitialPose())), 
                    driveTrain::getPose,
                    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                    driveTrain.getSimpleMotorFeedForward(),
                    driveTrain.getKinematics(),
                    driveTrain::getDriveWheelSpeeds,
                    driveTrain.getLeftPIDController(), 
                    driveTrain.getRightPIDController(),
                    driveTrain::setTankDriveVolts,
                    driveTrain
                )
            ),
            new AutoShootCmd(indexer, shooter, pickup)   
        );
    }
}