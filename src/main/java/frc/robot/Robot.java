package frc.robot;

import java.util.List;

import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.ArmControllCommands;
import frc.robot.commands.autonomous.AutonomousManager;
import frc.robot.commands.autonomous.ChargeAutonomous;
import frc.robot.commands.autonomous.FollowPath;
import frc.robot.commands.autonomous.PreDefPose;
import frc.robot.commands.autonomous.TimedForward;
import frc.robot.commands.balance.PIDBalanceCommand;
import frc.robot.commands.driveCommands.DriveCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.command.ParallelCommandGroup;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.robot.Constants.Drive.Autonomous;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.arm.Stop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Arm.ManualControlMode;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    PowerDistribution pdp = new PowerDistribution(62, ModuleType.kCTRE);

    boolean assumeArmAtHomeOnStart = false;

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
        Drive.getInstance().releaseBrake();

        if (assumeArmAtHomeOnStart) {
            Arm.getInstance().setEncoderTicksJoint(Arm.jointAngleToTicks(0));
            Arm.getInstance().setEncoderTicksBase(Arm.jointAngleToTicks(Math.toRadians(90)));
            Arm.getInstance().setManualControlMode(ManualControlMode.POS);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // if (pdp.getVoltage() < 8) {
        // System.out.println("Brown out!!!!!!!!!!!!!!!!!");
        // }
        //
        // if (pdp.getVoltage() < 8 &&
        // PneumaticHandler.getInstance().isCompressorPumping()) {
        // PneumaticHandler.getInstance().disableCompressor();
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

    static enum CurrentAutonomous {
        COOP_CUBE_TOP_CHARGE_STATION,
        RIGHT_CUBE_MOBILITY_PICKUP_POS,
        LEFT_CUBE_MOBILITY_PICKUP_POS,
        LEFT_CUBE_MOBILITY_PICKUP_CUBE
    }

    static final CurrentAutonomous autonomous = CurrentAutonomous.LEFT_CUBE_MOBILITY_PICKUP_CUBE;

    @Override
    public void autonomousInit() {
        Drive.getInstance().releaseBrake();
        Arm.getInstance().stop();
        Arm.getInstance().hold();

        //
        // Pose2d rp1 = PreDefPose.RedPiece1.toPose();
        // Pose2d target = new Pose2d(rp1.getX(), rp1.getY(), new Rotation2d(0, 1));
        // m_autonomousCommand = AutonomousManager.generatePathCommand(
        // List.of(PreDefPose.RedRightCorner.toPose(), PreDefPose.RedPiece4.toPose(),
        // PreDefPose.RedPiece1.toPose()));

        // Trajectory t = TrajectoryGenerator.generateTrajectory(
        // new Pose2d(0,0,new Rotation2d()),
        // List.of(new Translation2d(-1., 0),
        // new Translation2d(-0.5, 2.5)),
        // new Pose2d(-4.5, 2.5, new Rotation2d()),
        // Drive.getInstance().getTrajectoryConfig());
        //
        // m_autonomousCommand = new FollowPath(t);

        Command gotoTop = ArmControllCommands.gridForwardTop.construct();
        Command gotoPickup = ArmControllCommands.pickUpReverse.construct();
        Command gotoHome = ArmControllCommands.home.construct();

        Command openGripper = new InstantCommand(() -> GripperSubsystem.getInstance().openGripper());
        Command closedGripper = new InstantCommand(() -> GripperSubsystem.getInstance().closeGripper());

        if (autonomous == CurrentAutonomous.COOP_CUBE_TOP_CHARGE_STATION) {
            SequentialCommandGroup scoreCube = new SequentialCommandGroup(gotoTop, openGripper, new WaitCommand(0.5),
                    closedGripper);

            m_autonomousCommand = scoreCube
                    .andThen(new ParallelCommandGroup(gotoHome, new WaitCommand(2).andThen(new PIDBalanceCommand())));
        } else if (autonomous == CurrentAutonomous.RIGHT_CUBE_MOBILITY_PICKUP_POS) {
            SequentialCommandGroup scoreCube = new SequentialCommandGroup(gotoTop, openGripper, new WaitCommand(0.5),
                    closedGripper);

            CommandBase mobilityToCube = AutonomousManager
                    .getPathCommand(List.of(PreDefPose.RedRightCorner, PreDefPose.RedPiece4));

            m_autonomousCommand = scoreCube
                    .andThen(new ParallelCommandGroup(gotoHome, new WaitCommand(2).andThen(mobilityToCube)));
        } else if (autonomous == CurrentAutonomous.LEFT_CUBE_MOBILITY_PICKUP_POS) {
            SequentialCommandGroup scoreCube = new SequentialCommandGroup(gotoTop, openGripper, new WaitCommand(0.5),
                    closedGripper);

            CommandBase mobilityToCube = AutonomousManager
                    .getPathCommand(List.of(PreDefPose.RedRightCorner, PreDefPose.RedPiece4));

            m_autonomousCommand = scoreCube
                    .andThen(new ParallelCommandGroup(gotoHome, new WaitCommand(2).andThen(mobilityToCube)));
        } else if (autonomous == CurrentAutonomous.LEFT_CUBE_MOBILITY_PICKUP_CUBE) {
            SequentialCommandGroup scoreCube = new SequentialCommandGroup(gotoTop, openGripper, new WaitCommand(0.5),
                    closedGripper);

            CommandBase mobilityToCube = AutonomousManager
                    .getPathCommand(List.of(PreDefPose.RedRightCorner, PreDefPose.RedPiece4));

            m_autonomousCommand = scoreCube.andThen(new ParallelCommandGroup(
                    gotoHome.andThen(new WaitCommand(3))
                            .andThen(gotoPickup)
                            .andThen(openGripper),
                    new WaitCommand(2)
                            .andThen(mobilityToCube)))
                    .andThen(closedGripper)
                    .andThen(new WaitCommand(0.5))
                    .andThen(gotoHome);
        }

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
        Arm.getInstance().enableBreakModeJoint();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Drive.getInstance().reset();
        Drive.getInstance().releaseBrake();

        CommandScheduler.getInstance().schedule(new DriveCommand());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        Drive.getInstance().releaseBrake();
        CommandScheduler.getInstance().cancelAll();
        Arm.getInstance().stop();
        Arm.getInstance().hold();
    }

    @Override
    public void simulationInit() {
        Drive.getInstance().releaseBrake();
    }

    @Override
    public void simulationPeriodic() {
    }
}
