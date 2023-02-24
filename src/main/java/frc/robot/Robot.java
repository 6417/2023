// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.Navx;
import frc.robot.commands.autonomous.ChargeAutonomous;
import frc.robot.commands.autonomous.FollowPath;
import frc.robot.commands.autonomous.TimedForward;
import frc.robot.subsystems.drive.Drive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        Navx.setup(Port.kMXP);
        Navx.getInstance().init();
        JoystickHandler.getInstance()
                .setupJoysticks(List.of(Constants.Joystick.accelerator, Constants.Joystick.steeringWheel));
        JoystickHandler.getInstance().bind(Drive.getInstance());
        JoystickHandler.getInstance().init();

        Drive.getInstance().init();
        Shuffleboard.getTab("Debug").add(Drive.getInstance());
        // NetworkTableInstance.getDefault()
        //     .getTable("tools")
        //     .getTopic("chargeAutonomousCommand")
        //     .genericPublish("frc.robot.commands.autonomous.ChargeAutonomous", null);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

  @Override
  public void disabledPeriodic() {}

  TalonFX front_left;
  TalonFX front_right;
  TalonFX back_left;
  TalonFX back_right;

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // CommandScheduler.getInstance().schedule(new FollowPath("StraitRight"));
        CommandScheduler.getInstance().schedule(new TimedForward(1));

        // m_autonomousCommand = new ChargeAutonomous(StartingPosition.LEFT);
        // CommandScheduler.getInstance().schedule(new ChargeAutonomous(StartingPosition.LEFT));
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
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

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // double AnalogWert = analog.getValue();
        // double volt = AnalogWert*(5.0/4096);
        // double Distanz = 29.988 * Math.pow((volt), -1.173);
        // System.out.println(Distanz);

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
