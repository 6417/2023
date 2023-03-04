package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Constants.Drive.Motors;
import frc.robot.autonomous_tools.PathviewerLoader;
import frc.robot.autonomous_tools.RamseteCommandGenerator;
import frc.robot.commands.autonomous.ChargeAutonomous;
import frc.robot.commands.autonomous.FollowPath;
import frc.robot.commands.autonomous.TimedForward;
import frc.robot.commands.driveCommands.BalanceCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.util.List;

import javax.swing.JComboBox.KeySelectionManager;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.networktables.BooleanArrayTopic;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.networktables.TopicInfo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI.Port;
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

        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joysticks.armJoystick, Constants.Joystick.accelerator, Constants.Joystick.steeringWheel));
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


        FridoNavx.setup(Port.kMXP);
        FridoNavx.getInstance().init();
        JoystickHandler.getInstance()
            .setupJoysticks(List.of());
        JoystickHandler.getInstance().bind(Drive.getInstance());

        Drive.getInstance().init();
        Shuffleboard.getTab("Debug").add(Drive.getInstance());

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

    @Override
    public void autonomousInit() {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        // m_autonomousCommand = new ChargeAutonomous(StartingPosition.LEFT);
        m_autonomousCommand = new BalanceCommand();

        // var cmd = RamseteCommandGenerator.generateRamseteCommand(path);
        // CommandScheduler.getInstance().schedule(cmd);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Drive.getInstance().reset();
    }

    @Override
    public void teleopPeriodic() {
        // double AnalogWert = analog.getValue();
        // double volt = AnalogWert*(5.0/4096);
        // double Distanz = 29.988 * Math.pow((volt), -1.173);
        // System.out.println(Distanz);

    Drive.getInstance().drive(0.2,0,0);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Arm.getInstance().stop();
        Arm.getInstance().hold();
    }


    @Override
    public void simulationPeriodic() {
    }
}
