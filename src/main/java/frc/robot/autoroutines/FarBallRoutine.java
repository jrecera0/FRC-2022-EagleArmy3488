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

/** Add your docs here. */
public class FarBallRoutine {
    private Trajectory backupTrajectory;
    private Trajectory pickupTrajectory;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Indexer indexer;
    private Pickup pickup;
    private Pivot pivot;

    public FarBallRoutine(TrajectoryLoader trajectoryLoader, DriveTrain driveTrain, Shooter shooter, Indexer indexer, Pickup pickup, Pivot pivot) {
        // Common backup
        backupTrajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kBackupTraj);
        pickupTrajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kFarBallTraj);
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.pivot = pivot;
        this.pickup = pickup;
    }

    // something screwy is here because I can't make this a sequential command group...
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