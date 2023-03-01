package frc.robot;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private Trajectory path;

    @Override
    public void robotInit() {
        FridoNavx.setup(Port.kMXP);
        FridoNavx.getInstance().init();
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
        // path = PathviewerLoader.loadTrajectory("paths/LeftToChSt.wpilib.json");
        // for (int secs=0; secs<80; ++secs) {
        //     System.out.println(path.sample(secs).curvatureRadPerMeter);
        // }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
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

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Drive.getInstance().balance();
    Drive.getInstance().drive(0.2,0,0);
  }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
