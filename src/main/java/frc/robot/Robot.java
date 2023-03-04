// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.fridowpi.utils.Vector2;
import frc.robot.commands.arm.BaseGotoPositionShuffleBoard;
import frc.robot.subsystems.Arm;

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

    AnalogInput distanceSensor = new AnalogInput(0);

    private static boolean floatComp(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    static final double epsilon = 0.001;

    @Override
    public void robotInit() {
        Shuffleboard.getTab("debug").add("Base goto angle",
                BaseGotoPositionShuffleBoard.getInstance());

        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joysticks.armJoystick));
        PneumaticHandler.getInstance().configureCompressor(61,
                PneumaticsModuleType.CTREPCM);
        PneumaticHandler.getInstance().init();

        Arm.getInstance().init();

        JoystickHandler.getInstance().bind(Arm.getInstance());

        FridoDoubleSolenoid gripper = new FridoDoubleSolenoid(2, 3);
        gripper.init();
        // PneumaticHandler.getInstance().enableCompressor();
        // Shuffleboard.getTab("Arm").add(new ResetEncodersBase());
        Shuffleboard.getTab("debug").add("Analog In", distanceSensor);

        JoystickHandler.getInstance()
                .bind(new Binding(Constants.Joysticks.armJoystick, Logitech.rt, Button::toggleOnTrue,
                        new CommandBase() {
                            @Override
                            public void initialize() {
                                gripper.set(Value.kForward);
                            };

                            @Override
                            public boolean isFinished() {
                                return false;
                            }

                            @Override
                            public void end(boolean interrupted) {
                                gripper.set(Value.kReverse);
                            }

                            @Override
                            public Set<Subsystem> getRequirements() {
                                return new HashSet<>();
                            };
                        }));

        JoystickHandler.getInstance()
                .bind(new Binding(Constants.Joysticks.armJoystick, Logitech.x, Button::onFalse, new InstantCommand(
                        () -> Arm.getInstance().setEncoderTicksJoint(-167.0 / 360.0 /
                                Constants.Arm.jointGearRatio * 2048))));

        JoystickHandler.getInstance().init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        Arm.getInstance().stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        // schedule the autonomous command (example)
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
        Arm.getInstance().stop();
        Arm.getInstance().hold();
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
    }

    @Override
    public void testInit() {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
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
        Arm.getInstance().stop();
        Arm.getInstance().hold();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
