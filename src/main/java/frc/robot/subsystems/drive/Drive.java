// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.fridowpi.sensors.Navx;
import frc.robot.Constants;
import frc.robot.commands.driveCommands.DriveCommand;
import frc.robot.commands.driveCommands.ReverseDrivingDirection;
import frc.robot.commands.driveCommands.SetSteerMode;

public class Drive extends DriveBase {
    private static DriveBase instance;

    private Motors motors;

    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrive tankDrive;
    private LinearFilter driveFilter;
        
    private double driveDirection = 1;
    private double speed = 0.8;

    private double steerDirection = -1;
    private double steeringSensibility = 1;
    private SteerMode steerMode = Constants.Drive.defaultSteerMode;

    private PIDController rightVelocityController;
    private PIDController leftVelocityController;

    private SimpleMotorFeedforward motorFeedForward;

    private double maxVel = 0;

    public static enum SteerMode {
        CARLIKE, BIDIRECTIONAL
    }

    private Drive() {
        motors = new Motors();

        odometry = new DifferentialDriveOdometry(
            new Rotation2d(0),
            0.0, 0.0,
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        kinematics = new DifferentialDriveKinematics(
            Constants.Drive.Odometry.trackWidthMeters);
        
        motorFeedForward = new SimpleMotorFeedforward(
            Constants.Drive.PathWeaver.ksMeters, 
            Constants.Drive.PathWeaver.kvMetersPerSecoond,
            Constants.Drive.PathWeaver.ka);

        driveFilter = LinearFilter.movingAverage(Constants.Drive.movingAveragePrecision);
    }

    public static DriveBase getInstance() {
        if (instance != null) {
            return instance;
        }
        // If enabled, return a Drive object, else an empty DriveBase
        instance = Constants.Drive.enabled ? new Drive(): new DriveBase();
        return instance;
    }

    @Override
    public void init() {
        motors.init();
        setDefaultCommand(new DriveCommand());

        // configMotors();
        tankDrive = new DifferentialDrive(motors.left(), motors.right());

        resetSensors();

        // configSimpleMotorFeedforward();

        // configConstrains();

        // configTrajectoryConfig();

        rightVelocityController = new PIDController(Constants.Drive.PathWeaver.kP, Constants.Drive.PathWeaver.kI,
                Constants.Drive.PathWeaver.kD);
        leftVelocityController = new PIDController(Constants.Drive.PathWeaver.kP, Constants.Drive.PathWeaver.kI,
                Constants.Drive.PathWeaver.kD);

        System.out.println("Drive init completed");
    }

    @Override
    public void drive(double joystickInputY, double joystickTurnValue, double steerWheelInput) {
        double vel = Navx.getInstance().getVelocityX();
        if (vel > maxVel) {
            maxVel = vel;
            System.out.println(vel);
        }
        joystickInputY = deadZone(joystickInputY, 0.1);
        steerWheelInput = deadZone(steerWheelInput, 0.1);

        steerWheelInput *= steerDirection * steeringSensibility;
        joystickInputY *= driveDirection * speed;

        // If driving backwards in bidirectional mode, invert the steer direction
        if (joystickInputY < 0 && steerMode == SteerMode.CARLIKE)
            steerWheelInput = -steerWheelInput;

        Pair<Double, Double> pair = joystickToChassisSpeed(joystickInputY, steerWheelInput);
        tankDrive.arcadeDrive(pair.getFirst(), pair.getSecond(), false);
    }

    private Pair<Double, Double> joystickToChassisSpeed(double accelerationInput, double steerInput) {

        double velocity = driveFilter.calculate(accelerationInput);
        double steer = steerInput * Math.signum(accelerationInput) * 2;
        
        // Getting the sign of velocity and steer
        double velocitySign = Math.signum(velocity);
        double steerSign = Math.signum(steer);

        // Squaring the velocity, keeping it's sign
        velocity = Math.abs(velocity) * velocitySign;

        // Calculating the mapped steer and velocity values
        double mappedSteer = Math.min(Math.abs(steer), 1) * velocity * steerSign;
        double mappedVelocity = Math.min(Math.abs(Math.abs(velocity) + Math.abs(velocity) * Math.min(0, 1 - Math.abs(steer))), Math.abs(velocity)) * velocitySign;

        // Return with those values
        return new Pair<Double, Double>(mappedSteer, mappedVelocity);
    }

    private double deadZone(double val, double dead) {
        if (Math.abs(val) < dead)
            return 0.0;
        return (val -  Math.signum(val) * dead) / (1 - dead);
    }

    @Override
    public List<Binding> getMappings() {
        Binding driveForward = new Binding(Constants.Joystick.accelerator, LogitechExtreme._11, Button::toggleOnTrue, new ReverseDrivingDirection(false));
        Binding driveInverted = new Binding(Constants.Joystick.accelerator, LogitechExtreme._12, Button::toggleOnTrue, new ReverseDrivingDirection(true));
    
        Binding carMode = new Binding(Constants.Joystick.accelerator, LogitechExtreme._7, Button::toggleOnTrue, new SetSteerMode(SteerMode.CARLIKE));
        Binding bidirectionalMode = new Binding(Constants.Joystick.accelerator, LogitechExtreme._9, Button::toggleOnTrue, new SetSteerMode(SteerMode.BIDIRECTIONAL));
        return List.of(
            driveForward, driveInverted,
            carMode, bidirectionalMode);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("steer mode", () -> steerMode.name(), null);
        builder.addDoubleProperty("drive driection", () -> driveDirection, null);
        builder.addDoubleProperty("steering sensibility", () -> steeringSensibility, (val) -> steeringSensibility = val);
        builder.addDoubleProperty("speed", () -> speed, (val) -> speed = val);
    }

    @Override
    public void reverseDrivingDirection(boolean reverse) {
        driveDirection = reverse? -1: 1;
        steerDirection = reverse? 1: -1;
    }

    @Override
    public void setSteerMode(SteerMode mode) {
        steerMode = mode;
    }

    @Override
    public void setDirection(int direction) {
        // TODO: Direction enum/constants
        driveDirection = direction;
    }

    /* Getter Methods */

    @Override
    public Pose2d getPosition() {
        return odometry.getPoseMeters();
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                motors.left().getEncoderVelocity() * 10 / (Constants.Drive.Odometry.encoderToMetersConversion) * driveDirection,
                motors.right().getEncoderVelocity() * 10 / (Constants.Drive.Odometry.encoderToMetersConversion) * driveDirection);
    }

    @Override
    public void tankDriveVolts(Double leftSpeed, Double rightSpeed) {
        motors.left().setVoltage(-leftSpeed * driveDirection);
        motors.right().setVoltage(rightSpeed * driveDirection);
        tankDrive.feed();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    @Override
    public PIDController getRightVelocityController() {
        return leftVelocityController;
    }

    @Override
    public PIDController getLeftVelocityController() {
        return rightVelocityController;
    }

    @Override
    public DifferentialDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    @Override
    public SimpleMotorFeedforward getMotorFeedforward() {
        return motorFeedForward;
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public void stop() {
        tankDrive.stopMotor();
    }
}
