// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;

import java.util.ArrayList;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    final IJoystickId accelerator = () -> 0;
    final IJoystickId steeringWheel = () -> 1;

    FridolinsMotor frontRight = new FridoFalcon500(10);
    FridolinsMotor frontLeft = new FridoFalcon500(11);
    FridolinsMotor rearLeft = new FridoFalcon500(13);
    FridolinsMotor rearRight = new FridoFalcon500(12);

    final boolean steering = true;

    DifferentialDrive tankDrive;
    LinearFilter driveFilter;

    MotorControllerGroup right = new MotorControllerGroup(frontRight, rearRight);
    MotorControllerGroup left = new MotorControllerGroup(frontLeft, rearLeft);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        PowerDistribution pdp = new PowerDistribution(61, PowerDistribution.ModuleType.kCTRE);
        LiveWindow.enableAllTelemetry();
//        rearLeft.setInverted(true);
//        rearRight.setInverted(true);
        frontRight.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        frontLeft.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        rearLeft.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        rearRight.setIdleMode(FridolinsMotor.IdleMode.kBrake);

//        ((FridoFalcon500) frontRight).configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0.01));
//        ((FridoFalcon500) frontLeft).configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0.01));
//        ((FridoFalcon500) rearLeft).configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0.01));
//        ((FridoFalcon500) rearRight).configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0.01));


        JoystickHandler.getInstance().setupJoysticks(List.of(accelerator, steeringWheel));
        tankDrive = new DifferentialDrive(left, right);
        driveFilter = LinearFilter.movingAverage(20);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */

    public double deadZone(double val, double dead)  {
        if (Math.abs(val) > dead) {
            return (val -  Math.signum(val) * dead) / (1 -dead);
        } else {
            return 0.0;
        }
    }

    TalonFX motor = new TalonFX(0);

    @Override
    public void robotPeriodic() {
        // Getting the steer values from the joystick and the steering wheel

       if (steering) {
           double speed = 1;
           double driveDirection = 1;
           double velocity = driveFilter.calculate(JoystickHandler.getInstance().getJoystick(accelerator).getY()) * speed * driveDirection;
           double steer = JoystickHandler.getInstance().getJoystick(steeringWheel).getX() * Math.signum(JoystickHandler.getInstance().getJoystick(accelerator).getY()) * 2 * driveDirection;

           // Getting the sign of velocity and steer
           double velocitySign = Math.signum(velocity);
           double steerSign = Math.signum(steer);

           // Squaring the velocity, keeping it's sign
           velocity = Math.abs(velocity) * velocitySign;

           // Calculating the mapped steer and velocity values
           double mappedSteer = Math.min(Math.abs(steer), 1) * velocity * steerSign;
           double mappedVelocity = Math.min(Math.abs(Math.abs(velocity) + Math.abs(velocity) * Math.min(0, 1 - Math.abs(steer))), Math.abs(velocity)) * velocitySign;

           // Driving with those values
           tankDrive.arcadeDrive(mappedSteer,
                   mappedVelocity, false);
       } else {
           right.set(deadZone(JoystickHandler.getInstance().getJoystick(accelerator).getY(), 0.1));
           left.set(deadZone(-JoystickHandler.getInstance().getJoystick(steeringWheel).getY(), 0.1));
       }

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
