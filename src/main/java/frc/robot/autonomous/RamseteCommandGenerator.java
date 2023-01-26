package frc.robot.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drive;

public class RamseteCommandGenerator {
    public static Command generateRamseteCommand(Trajectory path) {
        // Resetting all the sensors and the odometry on the robot and setting the initial pose
        Drive.getInstance().resetSensors();
        Drive.getInstance().resetOdometry(path.getInitialPose());

        // Generating the ramsete command
        RamseteCommand ramseteCommand = new RamseteCommand(
        path, 
        Drive.getInstance()::getPosition,
        new RamseteController(Drive.Constants.PathWeaver.kRamseteB, Drive.Constants.PathWeaver.kRamseteZeta),
        Drive.getInstance().getMotorFeedforward(),
        Drive.getInstance().getDriveKinematics(),
        Drive.getInstance()::getWheelSpeeds,
        Drive.getInstance().getLeftVelocityController(),
        Drive.getInstance().getRightVelocityController(),
        (leftSpeed, rightSpeed) -> {
            Drive.getInstance().tankDriveVolts(leftSpeed, rightSpeed);
        },
        Drive.getInstance());

        // Finishing and returning the command
        return ramseteCommand.andThen(() -> Drive.getInstance().stop());
    }
}