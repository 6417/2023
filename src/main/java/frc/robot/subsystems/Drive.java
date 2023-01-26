// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.motors.FridoFalcon500;
import frc.robot.Constants;

public class Drive extends DriveBase {
    private static DriveBase instance;
    private FridoFalcon500 mBackLeft;
    private FridoFalcon500 mBackRight;
    private FridoFalcon500 mFrontLeft;
    private FridoFalcon500 mFrontRight;

    private MotorControllerGroup motorsLeft;
    private MotorControllerGroup motorsRight;

    private DifferentialDrive drive;

    /** Constructor */
    private Drive() {
        // TODO: test which motor is which (naming currently not reliable!)
        mBackLeft = new FridoFalcon500(0);
        mBackRight = new FridoFalcon500(1);
        mFrontLeft = new FridoFalcon500(2);
        mFrontRight = new FridoFalcon500(3);
        
        motorsLeft = new MotorControllerGroup(mBackLeft, mFrontLeft);
        motorsRight = new MotorControllerGroup(mBackRight, mFrontRight);

        drive = new DifferentialDrive(motorsLeft, motorsRight);
    }

    public static DriveBase getInstance() {
        if (instance != null) {
            return instance;
        }
        // For debugging:
        // If enabled, return a Drive object, else an empty DriveBase
        instance = Constants.Drive.enabled? new Drive(): new DriveBase();
        return instance;
    }

    public void drive(double x, double y) {
        Pair<Double, Double> pair = this.joystickToChassisSpeed(x, y);
        drive.arcadeDrive(pair.getFirst(), pair.getSecond(), false);
    }

    private Pair<Double, Double> joystickToChassisSpeed(double x, double y) {
        double speed = 1;
        double driveDirection = 1;
        LinearFilter driveFilter = LinearFilter.movingAverage(Constants.Drive.movingAveragePrecision);
        double velocity = driveFilter.calculate(JoystickHandler.getInstance().getJoystick(Constants.Drive.accelerator).getY()) * speed * driveDirection;
        double steer = JoystickHandler.getInstance().getJoystick(Constants.Drive.steeringWheel).getX() * Math.signum(JoystickHandler.getInstance().getJoystick(Constants.Drive.accelerator).getY()) * 2 * driveDirection;

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

    @Override
    public void periodic() { }

    @Override
    public void simulationPeriodic() { }
}
