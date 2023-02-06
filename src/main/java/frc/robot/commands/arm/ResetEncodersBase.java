package frc.robot.commands.arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetEncodersBase extends CommandBase {
    @Override
    public void initialize() {
        Arm.getInstance().resetEncodersBase();
    }
}
