// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive.SteerMode;

public class DriveBase extends SubsystemBase {

    public void drive(double x, double y) { }

    public void init() { }

    public void reverseDrivingDirection(boolean reverse) { }

    public void setSteerMode(SteerMode mode) { }
    
    public Pose2d getPosition() { 
        return null;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return null;
    }

    public void tankDriveVolts(Double leftSpeed, Double rightSpeed) { }

    public ChassisSpeeds getChassisSpeeds() {
        return null;
    }

    public PIDController getRightVelocityController() {
        return null;
    }

    public PIDController getLeftVelocityController() {
        return null;
    }

    public DifferentialDriveKinematics getDriveKinematics() {
        return null;
    }

    public SimpleMotorFeedforward getMotorFeedforward() {
        return null;
    }

    public void stop() { }

    public void resetSensors() { }

    public void resetOdometry(Pose2d initialPose) { }

    @Override
    public void periodic() { }

    @Override
    public void simulationPeriodic() { }
}
