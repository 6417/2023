// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.JoystickBindable;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.robot.ArmModel.ArmStateSupplier;
import frc.robot.ArmModel.Cargo;
import frc.robot.commands.arm.HoldAtCurrentPosition;
import frc.robot.commands.arm.ResetEncodersBase;
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

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    FridoFalcon500 baseMaster = new FridoFalcon500(20);
    FridoFalcon500 baseFollower = new FridoFalcon500(21);
    FridoFalcon500 jointMaster = new FridoFalcon500(22);
    FridoFalcon500 jointFollower = new FridoFalcon500(23);
    ArmModel armModel = new ArmModel(
            new ArmStateSupplier(
                    () -> Math.asin(Math
                            .sin(baseMaster.getEncoderTicks() * Constants.Arm.baseGearRatio / 2048 * Math.PI * 2.0)),
                    () -> (jointMaster.getEncoderTicks() * Constants.Arm.jointGearRatio / 2048 * Math.PI * 2.0
                            + Math.PI) % (2.0 * Math.PI) - Math.PI,
                    () -> baseMaster.getEncoderVelocity(), // * Constants.Arm.baseGearRatio / 2048 * Math.PI * 2.0 * 10,
                    () -> jointMaster.getEncoderVelocity()), // * Constants.Arm.jointGearRatio / 2048 * Math.PI * 2.0 * 10),
            Cargo.None);

    private final IJoystickId armJoystick = () -> 0;

    AnalogInput distanceSensor = new AnalogInput(0);

    @Override
    public void robotInit() {
        // Arm.getInstance().init();
        // Arm.getInstance().setDefaultCommand(new HoldAtCurrentPosition());

        JoystickHandler.getInstance().setupJoysticks(List.of(armJoystick));
        PneumaticHandler.getInstance().configureCompressor(61, PneumaticsModuleType.CTREPCM);
        PneumaticHandler.getInstance().init();

        FridoDoubleSolenoid gripper = new FridoDoubleSolenoid(2, 3);
        gripper.init();
        // PneumaticHandler.getInstance().enableCompressor();
        // Shuffleboard.getTab("Arm").add(new ResetEncodersBase());

        jointMaster.factoryDefault();
        baseMaster.factoryDefault();
        baseFollower.factoryDefault();
        jointFollower.factoryDefault();

        jointFollower.follow(jointMaster, DirectionType.invertMaster);
        jointFollower.setInverted(true);
        baseFollower.follow(baseMaster, DirectionType.followMaster);

        baseMaster.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed, true); // TODO:
        baseFollower.enableForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed, false); // TODO:

        baseMaster.enableReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed, true); // TODO:
        baseFollower.enableReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed, true); // TODO:

        jointFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                1.5 * Constants.Arm.torqueToAmpsProportionality, 1.5 * Constants.Arm.torqueToAmpsProportionality, 1));
        jointMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                1.5 * Constants.Arm.torqueToAmpsProportionality, 1.5 * Constants.Arm.torqueToAmpsProportionality, 1));
        baseMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                1.5 * Constants.Arm.torqueToAmpsProportionality, 1.5 * Constants.Arm.torqueToAmpsProportionality, 1));
        baseFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                1.5 * Constants.Arm.torqueToAmpsProportionality, 1.5 * Constants.Arm.torqueToAmpsProportionality, 1));

        baseMaster.setIdleMode(IdleMode.kBrake);
        baseFollower.setIdleMode(IdleMode.kBrake);

        jointMaster.setIdleMode(IdleMode.kBrake);
        jointFollower.setIdleMode(IdleMode.kBrake);

        Shuffleboard.getTab("debug").addDouble("base right current stator", baseMaster::getStatorCurrent);
        Shuffleboard.getTab("debug").addDouble("base left current stator", baseFollower::getStatorCurrent);
        Shuffleboard.getTab("debug").addDouble("Joystick after dead zone",
                () -> deadZone(JoystickHandler.getInstance().getJoystick(armJoystick).getY(), 0.1));

        Shuffleboard.getTab("debug").addDouble("base right speed [RPS]",
                () -> baseMaster.getEncoderVelocity() / 2048 * 10);
        Shuffleboard.getTab("debug").addDouble("base left speed [RPS]",
                () -> baseMaster.getEncoderVelocity() / 2048 * 10);

        Shuffleboard.getTab("debug").addDouble("joint right speed [RPS]",
                () -> jointMaster.getEncoderVelocity() / 2048 * 10);
        Shuffleboard.getTab("debug").addDouble("joint left speed [RPS]",
                () -> jointMaster.getEncoderVelocity() / 2048 * 10);

        Shuffleboard.getTab("debug").addDouble("joint speed [RAW]",
                () -> jointMaster.getEncoderVelocity());

        Shuffleboard.getTab("debug").addBoolean("base right fw", () -> baseMaster.getForwardLimitSwitch().get());
        Shuffleboard.getTab("debug").addBoolean("base right rev", () -> baseMaster.getReverseLimitSwitch().get());
        Shuffleboard.getTab("debug").addBoolean("base left fw", () -> baseFollower.getForwardLimitSwitch().get());
        Shuffleboard.getTab("debug").addBoolean("base left rev", () -> baseFollower.getReverseLimitSwitch().get());

        Shuffleboard.getTab("debug").addDouble("base right position [DEG]",
                () -> baseMaster.getSelectedSensorPosition() * Constants.Arm.baseGearRatio / 2048 * 360);
        Shuffleboard.getTab("debug").addDouble("base left position [DEG]",
                () -> baseFollower.getSelectedSensorPosition() * Constants.Arm.baseGearRatio / 2048 * 360);

        Shuffleboard.getTab("debug").addDouble("base right position [RAW]",
                () -> baseMaster.getSelectedSensorPosition());
        Shuffleboard.getTab("debug").addDouble("base left position [RAW]",
                () -> baseFollower.getSelectedSensorPosition());
        Shuffleboard.getTab("debug").add("Arm Model", armModel);
        Shuffleboard.getTab("debug").addDouble("Distance Sensor [RAW]", distanceSensor::getValue);

        Shuffleboard.getTab("debug").addDouble("gripper position [RAW]", jointMaster::getEncoderTicks);
        Shuffleboard.getTab("debug").addDouble("gripper position [DEG]",
                () -> armModel.state.gripperArmAngle.get() * 180 / Math.PI);

        Shuffleboard.getTab("debug").addDouble("joint stator current [A]", jointMaster::getStatorCurrent);
        Shuffleboard.getTab("debug").addDouble("base speed [RAW]", baseMaster::getEncoderVelocity);

        JoystickHandler.getInstance().bind(new Binding(armJoystick, Logitech.rt, Button::toggleOnTrue, new Command() {
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
                // TODO Auto-generated method stub
                return new HashSet<>();
            };
        }));

        JoystickHandler.getInstance().init();

        baseFollower.configEncoder(FridoFeedBackDevice.kBuildin, 2048);
        baseMaster.configEncoder(FridoFeedBackDevice.kBuildin, 2048);
        jointMaster.configEncoder(FridoFeedBackDevice.kBuildin, 2048);
        jointFollower.configEncoder(FridoFeedBackDevice.kBuildin, 2048);

        baseMaster.setInverted(true);
        baseFollower.setInverted(true);

        JoystickHandler.getInstance().bind(new Binding(armJoystick, Logitech.x, Button::onFalse, new InstantCommand(
                () -> jointMaster.setEncoderPosition(-167.0 / 360.0 / Constants.Arm.jointGearRatio * 2048))));

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
    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    double deadZone(double in, double threashhold) {
        if (Math.abs(in) > threashhold) {
            return Math.signum(in) * map(Math.abs(in), threashhold, 1, 0, 1);
        } else {
            return 0.0;
        }

    }

    @Override
    public void robotPeriodic() {
        baseMaster.set(0.3 * deadZone(JoystickHandler.getInstance().getJoystick(armJoystick).getY(), 0.1));
        jointMaster.set(0.3 * deadZone(JoystickHandler.getInstance().getJoystick(armJoystick).getThrottle(), 0.1));

        if (!baseMaster.isReverseLimitSwitchActive()) {
            baseMaster.setEncoderPosition(30.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0);
        }

        if (!baseMaster.isForwardLimitSwitchActive()) {
            baseMaster.setEncoderPosition(150.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0);
        }

        if (!baseFollower.isReverseLimitSwitchActive()) {
            baseFollower.setEncoderPosition(30.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0);
        }

        if (!baseFollower.isForwardLimitSwitchActive()) {
            baseFollower.setEncoderPosition(150.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0);
        }

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
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
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
