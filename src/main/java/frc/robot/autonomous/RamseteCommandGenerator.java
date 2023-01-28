package frc.robot.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;

public class RamseteCommandGenerator {
    public static Command generateRamseteCommand(Trajectory path) {
        // Get a pointer to the Drive singleton
        DriveBase instance = Drive.getInstance();

        // Resetting all the sensors and the odometry on the robot and setting the initial pose
        instance.resetSensors();
        instance.resetOdometry(path.getInitialPose());

        // Generating the ramsete command
        RamseteCommand ramseteCommand = new RamseteCommand(
            path, 
            instance::getPosition,
            new RamseteController(Constants.Drive.PathWeaver.kRamseteB, Constants.Drive.PathWeaver.kRamseteZeta),
            instance.getMotorFeedforward(),
            instance.getDriveKinematics(),
            instance::getWheelSpeeds,
            instance.getLeftVelocityController(),
            instance.getRightVelocityController(),
            (leftSpeed, rightSpeed) -> {
                instance.tankDriveVolts(leftSpeed, rightSpeed);
            },
            instance
        );

        // Finishing and returning the command
        return ramseteCommand.andThen(() -> instance.stop());
    }
}