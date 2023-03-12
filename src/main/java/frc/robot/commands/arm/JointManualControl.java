package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class JointManualControl extends CommandBase {
    private static double maxPercent = Constants.Arm.jointDefaultManualMaxPercent;
    static {
        Shuffleboard.getTab("Params").add("Joint ManualControl", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("joint manual control max output", () -> maxPercent,
                        (newMaxPercent) -> maxPercent = MathUtil.clamp(newMaxPercent, -1, 1));
            }
        });
    }
    
    @Override
    public void initialize() {
        Arm.getInstance().jointGotoAngle(Arm.getInstance().jointAngle());
    }

    @Override
    public void execute() {
        double percent = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getThrottle(),
                Constants.Joysticks.armJoystickDeadZone);
        if (Arm.getInstance().isZeroed()) {
            Arm.getInstance()
                    .jointGotoAngle(Arm.getInstance().getJointTargetAngle() + Arm.jointTicksToAngle(150) * percent);
        }
    }
}