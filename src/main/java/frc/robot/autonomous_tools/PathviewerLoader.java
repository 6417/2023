package frc.robot.autonomous_tools;

import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class PathviewerLoader {
    public static Trajectory loadTrajectory(String path) {
        // Declaring variables
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath;

        try {
            // Generating the filepath to the trajectory file
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);

            // Loading the trajectory from the file
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException e) {
                DriverStation.reportError("Wrong path: " + path, e.getStackTrace());
            }
        } catch (InvalidPathException e) {
            // Throwing an exception in case it files
            DriverStation.reportError("Wrong path: " + path, e.getStackTrace());
        }

        return trajectory;
    }
}