package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.apache.logging.log4j.core.layout.SyslogLayout;

import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.ArmPathGenerator.RobotOrientation;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.Constants.Gripper;
import frc.robot.commands.driveCommands.BalanceCommand;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.fridowpi.utils.Vector2;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.arm.BaseGotoPositionShuffleBoard;
import frc.robot.commands.arm.Stop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GripperSubsystem;

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

    PowerDistribution pdp = new PowerDistribution(62, ModuleType.kCTRE);

    @Override
    public void robotInit() {
        JoystickHandler.getInstance().setJoystickFactory(ArmPosJoystick::new);
        Shuffleboard.getTab("debug").add("Base goto angle",
                BaseGotoPositionShuffleBoard.getInstance());

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

        if (pdp.getVoltage() < 8) {
            System.out.println("Brown out!!!!!!!!!!!!!!!!!");
        }

        if (pdp.getVoltage() < 8  && PneumaticHandler.getInstance().isCompressorPumping()) {
            PneumaticHandler.getInstance().disableCompressor();
        }
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
        CommandScheduler.getInstance().schedule(new Stop());
        Arm.getInstance().stop();
        Arm.getInstance().hold();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Drive.getInstance().reset();
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
    public void simulationPeriodic() {
    }
}