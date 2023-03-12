package frc.robot;

import java.util.List;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.commands.autonomous.AutonomousManager;
import frc.robot.commands.driveCommands.DriveCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.arm.Stop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GripperSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    PowerDistribution pdp = new PowerDistribution(62, ModuleType.kCTRE);

    @Override
    public void robotInit() {
        JoystickHandler.getInstance().setJoystickFactory(ArmPosJoystick::new);
        Shuffleboard.getTab("debug").add(CommandScheduler.getInstance());

        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joysticks.armJoystick,
                Constants.Joysticks.accelerator, Constants.Joysticks.steeringWheel));
        PneumaticHandler.getInstance().configureCompressor(61,
                PneumaticsModuleType.CTREPCM);
        PneumaticHandler.getInstance().init();

        Arm.getInstance().init();

        JoystickHandler.getInstance().bind(Arm.getInstance());
        JoystickHandler.getInstance().bind(GripperSubsystem.getInstance());

        // PneumaticHandler.getInstance().enableCompressor();
        // Shuffleboard.getTab("Arm").add(new ResetEncodersBase());

        FridoNavx.setup(Port.kMXP);
        FridoNavx.getInstance().init();
        JoystickHandler.getInstance()
                .setupJoysticks(List.of());
        JoystickHandler.getInstance().bind(Drive.getInstance());

        Drive.getInstance().init();
        GripperSubsystem.getInstance().init();
        Shuffleboard.getTab("Debug").add(Drive.getInstance());

        JoystickHandler.getInstance().bind(new Binding(Constants.Joysticks.accelerator, LogitechExtreme._8,
                Button::toggleOnTrue, new ToggleCompressor()));

        JoystickHandler.getInstance().init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // if (pdp.getVoltage() < 8) {
        //     System.out.println("Brown out!!!!!!!!!!!!!!!!!");
        // }
        //
        // if (pdp.getVoltage() < 8  && PneumaticHandler.getInstance().isCompressorPumping()) {
        //     PneumaticHandler.getInstance().disableCompressor();
        // }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        Arm.getInstance().stop();
        Arm.getInstance().enableBreakModeJoint();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        // m_autonomousCommand = new ChargeAutonomous(StartingPosition.LEFT);
        m_autonomousCommand = AutonomousManager.getCommand_DriveOnCharginStation();

        // var cmd = RamseteCommandGenerator.generateRamseteCommand(path);
        // CommandScheduler.getInstance().schedule(cmd);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            CommandScheduler.getInstance().onCommandFinish(
                (cmd) -> CommandScheduler.getInstance().schedule(new DriveCommand()));
        }
    }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new Stop());
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        Arm.getInstance().enableBreakModeJoint();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Drive.getInstance().reset();

        CommandScheduler.getInstance().schedule(new DriveCommand());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Arm.getInstance().stop();
        Arm.getInstance().hold();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
