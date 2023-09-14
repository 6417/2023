package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.utils.LatchedBooleanRising;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmKinematics;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ManualPosControl extends CommandBase {
    private Vector2 target;
    private LatchedBooleanRising gettingZeroed;

    public ManualPosControl() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        target = Arm.getInstance().getPos();
        gettingZeroed = new LatchedBooleanRising(Arm.getInstance().isZeroed());
    }

    @Override
    public void execute() {
        if (gettingZeroed.updateAndGet(Arm.getInstance().isZeroed())) {
            target = Arm.getInstance().getPos();
        }

        double px =  (Arm.getInstance().xDirInverted ? 1 : -1) * -MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getY(),
                Constants.Joysticks.armJoystickDeadZone);

        double py = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getThrottle(),
                Constants.Joysticks.armJoystickDeadZone);

        if (Math.abs(px) > 0.0001 || Math.abs(py) > 0.0001) {
            if (Arm.getInstance().isZeroed()) {
                Vector2 newTarget = target.add(new Vector2(px, py).smul(-0.005));
                if (!Arm.getInstance().isPosValid(newTarget)) {
                    DriverStation.reportError("[ManualPosControl::execute] Invalid position", false);
                    return;
                }
                target = newTarget;
                var solutions = ArmKinematics.posToAngles(target);
                ArmKinematics.Values<Double> angles;

                if (target.x >= 0) {
                    angles = solutions.getFirst();
                } else {
                    angles = solutions.getSecond();
                }

                Arm.getInstance().baseGotoAngle(angles.base);
                Arm.getInstance().jointGotoAngle(angles.joint);
            }
        }
    }
}
