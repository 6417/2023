package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RawManualControl extends CommandBase {
    public RawManualControl() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void execute() {
        double px = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getY(),
                Constants.Joysticks.armJoystickDeadZone);

        double py = MathUtil.applyDeadband(
                JoystickHandler.getInstance().getJoystick(Constants.Joysticks.armJoystick).getThrottle(),
                Constants.Joysticks.armJoystickDeadZone);
        Arm.getInstance().setBasePercent(px * 0.3);
        Arm.getInstance().setJointPercent(py * 0.3);
    }
}
