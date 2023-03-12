package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Arm;

public class ZeroJoint extends SequentialCommandGroup {
    private static class ZeroJointInBaseArmPos extends CommandBase {
        @Override
        public void execute() {
            Arm.getInstance().setJointPercent(-0.05);
        }

        @Override
        public boolean isFinished() {
            return Arm.getInstance().isJointZeroed();
        }
    }

    private static class BaseGotoAngle extends CommandBase {
        @Override
        public void initialize() {
            Arm.getInstance().baseGotoAngleNoZeroedCheckOfJoint(Math.toRadians(97.16));
        }

        @Override
        public boolean isFinished() {
            return Arm.getInstance().isBaseAtTarget();
        }

        @Override
        public void end(boolean interrupted) {
            Arm.getInstance().stop();
        }
    }

    private static class CancelOnBaseNotZeroed extends CommandBase {
        @Override
        public void initialize() {
        }

        @Override
        public boolean isFinished() {
            return !Arm.getInstance().isBaseZeroed();
        }
    }

    public ZeroJoint() {
        addRequirements(Arm.getInstance());
        addCommands(
                new ParallelRaceGroup(new CancelOnBaseNotZeroed(), new SequentialCommandGroup(new InstantCommand(() -> {
                    System.out.println("disableBreakModeJoint InstantCommand");
                    Arm.getInstance().disableBreakModeJoint();
                }), new BaseGotoAngle(), new ZeroJointInBaseArmPos())), new InstantCommand(() -> {
                    System.out.println("enableBreakModeJoint InstantCommand");
                    Arm.getInstance().enableBreakModeJoint();
                }));
    }
}
