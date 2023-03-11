package frc.robot;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.commands.autonomous.AutonomousManager;
import frc.robot.commands.driveCommands.DriveCommand;
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
        // m_autonomousCommand = new PIDBalanceCommand();
        m_autonomousCommand = AutonomousManager.getCommand_DriveOnCharginStation();

        // var cmd = RamseteCommandGenerator.generateRamseteCommand(path);
        // CommandScheduler.getInstance().schedule(cmd);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Drive.getInstance().reset();

        CommandScheduler.getInstance().schedule(new DriveCommand());
    }

    @Override
    public void teleopPeriodic() {
        // double AnalogWert = analog.getValue();
        // double volt = AnalogWert*(5.0/4096);
        // double Distanz = 29.988 * Math.pow((volt), -1.173);
        // System.out.println(Distanz);

        // Drive.getInstance().balance();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
