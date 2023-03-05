package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class BaseManualControl extends CommandBase {
    private static double maxPercent = Constants.Arm.baseDefaultManualMaxPercent;
    static {
        Shuffleboard.getTab("Params").add("Base ManualControl", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("base manual control max output", () -> maxPercent,
                        (newMaxPercent) -> maxPercent = MathUtil.clamp(newMaxPercent, -1, 1));
            }
        });
    }

    @Override
    public void execute() {
        double percent = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getY(),
                Constants.Joysticks.armJoystickDeadZone);

        if (Arm.getInstance().isZeroed()) {
            Arm.getInstance()
                    .baseGotoAngle(Arm.getInstance().getBaseTargetAngle() + Arm.baseTicksToAngle(150) * percent);
        }
    }
}
