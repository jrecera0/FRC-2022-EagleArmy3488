// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.management;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * A custom class to load trajectories that are put at the root of the
 * deploy directory. To a certain degree, this is somewhat cursed because
 * loops are a bit of a controversial and iffy thing to use for frc code,
 * but for the sake of being able to load many files on robot code start,
 * this should be appropriate.
 **/
public class TrajectoryLoader {
    // Create dictionary map for trajectories and their names
    private Map<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    // Attempt to load all trajectories in the root deploy folder
    public TrajectoryLoader() {
        String[] trajectoryFilepathStrings = Filesystem.getDeployDirectory().list();
        for (int i = 0; i < trajectoryFilepathStrings.length; i++) {
            try {
                Path trajFilepath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilepathStrings[i]);
                trajectories.put(trajectoryFilepathStrings[i], TrajectoryUtil.fromPathweaverJson(trajFilepath));
                System.out.println("Trajectory " + trajFilepath.toString() + " has loaded!");
            } catch (IOException ex) {
                DriverStation.reportError(ex.getMessage(), true);
            }
        }
    }

    // Getter for trajectories by their filename
    public Trajectory getTrajectory(String trajectoryName) {
        return trajectories.get(trajectoryName);
    }

    // If we need to string trajectories together, we do it here...
    public void concatenateTrajectories(String... trajectoryNames) {
        // Parent trajectory that trajectories will be added to
        Trajectory parentTrajectory = null;
        String parentTrajectoryName = null;
        
        // For-loop in order to interate through each one of the passed in
        // trajectory names in order, with each one of those being checked
        // against the dictionary.
        for (int i = 0; i < trajectoryNames.length; i++) {
            // Get temp traj and see if it is in the dictionary
            Trajectory traj = trajectories.get(trajectoryNames[i]);

            // check if we do have a valid trajectory
            if (traj != null) {
                // If there is no parent trajectory, make one
                if (parentTrajectory == null) {
                    parentTrajectory = traj;
                    parentTrajectoryName = trajectoryNames[i];
                }
                // Else concatinate the remaining passed in trajectories to the parent
                else {
                    trajectories.replace(parentTrajectoryName, parentTrajectory.concatenate(traj));
                }
            }
        }
    }
}
