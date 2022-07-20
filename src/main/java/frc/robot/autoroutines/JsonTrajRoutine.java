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

/**
 * Custom autoroutine class that can be primarily used for testing. The same notes from
 * the other autoroutines are applicable here. I highly recommend writing test classes
 * or separated autoroutines before integrating them into a sort of "final routine" that
 * will be used for competition. This way you don't end up cluttering up your final robot
 * project so much.
 */
public class JsonTrajRoutine {
    private Trajectory trajectory;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Indexer indexer;
    private Pickup pickup;
    private Pivot pivot;
    
    /**
     * Use this to set up the {@code JsonTrajRoutine} with all required subsystems and the
     * {@link TrajectoryLoader} to load trajectories. The parameters of this can and will change
     * depending on what it is you are trying to test.
     * 
     * @param trajectoryLoader Trajectory loader to grab the trajectories
     * @param driveTrain {@code DriveTrain} to be used for driving the trajectories
     * @param shooter {@code Shooter} to be used for shooting power cells
     * @param indexer {@code Indexer} to be used to queue power cells towards the {@code Shooter}
     * @param pickup {@code Pickup} to be used to pick up power cells from the field
     * @param pivot {@code Pivot} to be used to raise and lower the pickup throughout the routine
     */
    public JsonTrajRoutine(DriveTrain driveTrain, TrajectoryLoader trajectoryLoader, Shooter shooter, Indexer indexer, Pickup pickup, Pivot pivot) {
        trajectory = trajectoryLoader.getTrajectory(TrajectoryPathnames.kJsonTrajPath);
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.pickup = pickup;
        this.pivot = pivot;
    }

    /**
     * Returns the assembled {@link SequentialCommandGroup} with the entire autonomous routine.
     * This is what you realistically want to be editing when you're making changes to the routine
     * as a whole, outside of changing trajectories.
     * @return {@code JsonTrajRoutine} {@link Command} 
     */
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
