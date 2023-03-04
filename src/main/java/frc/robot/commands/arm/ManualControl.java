package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.utils.LatchBooleanRising;
import frc.fridowpi.utils.LatchedBoolean;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmKinematics;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ManualControl extends CommandBase {
    private Vector2 target;
    private LatchBooleanRising gettingZeroed; 
    public ManualControl() {
        addRequirements(Arm.getInstance());
    }
    
    @Override
    public void initialize() {
        target = Arm.getInstance().getPos();
        gettingZeroed = new LatchBooleanRising(Arm.getInstance().isZeroed());
    }

    @Override
    public void execute() {
        if (gettingZeroed.updateAndGet(Arm.getInstance().isZeroed())) {
            target = Arm.getInstance().getPos();
        }

        double px = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getY(),
                Constants.Joysticks.armJoystickDeadZone);

        double py = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getThrottle(),
                Constants.Joysticks.armJoystickDeadZone);

        if (Arm.getInstance().isZeroed()) {
            target  = target.add(new Vector2(px, py).smul(-0.005));
            var solutions = ArmKinematics.posToAngles(target);
            ArmKinematics.Values<Double> angles;

            if (target.x >= 0) {
                angles = solutions.getFirst();
            } else {
                angles = solutions.getSecond();
            }

            Arm.getInstance().baseGotoAngle(angles.base);
            Arm.getInstance().jointGotoAngle(angles.joint);
        } else {
            Arm.getInstance().setBasePercent(px * 0.3);
            Arm.getInstance().setJointPercent(py * 0.3);
        }
    }
}
