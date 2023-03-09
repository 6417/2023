package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmKinematics;
import frc.robot.subsystems.Arm;

public class GotoPosNoChecksDriveThrough extends CommandBase {
    Vector2 target;

    public GotoPosNoChecksDriveThrough(Vector2 target) {
        this.target = target;

        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        var solutions = ArmKinematics.posToAngles(target);
        ArmKinematics.Values<Double> angles;

        if (target.x >= 0) {
            angles = solutions.getFirst();
        } else {
            angles = solutions.getSecond();
        }

        Arm.getInstance().baseGotoAngle(angles.base, false);
        Arm.getInstance().jointGotoAngle(angles.joint, false);
    }

    @Override
    public void end(boolean interrupted) {
        Arm.getInstance().stop();
        Arm.getInstance().hold();
    }

    @Override
    public boolean isFinished() {
        return Arm.getInstance().isArmAtTarget();
    }
}
