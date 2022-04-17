// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoroutines;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryPathnames;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.DropPivotCmd;
import frc.robot.management.TrajectoryLoader;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class JsonTrajRoutine {
    private Trajectory trajectory;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Indexer indexer;
    private Pickup pickup;
    private Pivot pivot;
    
    public JsonTrajRoutine(DriveTrain driveTrain, TrajectoryLoader trajectoryLoader, Shooter shooter, Indexer indexer, Pickup pickup, Pivot pivot) {
        trajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kJsonTrajPath);
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.pickup = pickup;
        this.pivot = pivot;
    }
    
    public Command getCommand() {
        return new SequentialCommandGroup(
            new AutoShootCmd(indexer, shooter, pickup).withTimeout(1.5),
            new RamseteCommand(
                trajectory.transformBy(driveTrain.getPose().minus(trajectory.getInitialPose())), 
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
            new DropPivotCmd(pivot)
        );
        
    }
}
