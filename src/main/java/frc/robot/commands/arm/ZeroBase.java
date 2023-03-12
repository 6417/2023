package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroBase extends CommandBase {
    public ZeroBase() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        if (!isFinished()) {
            Arm.getInstance().setBasePercent(0.05);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Arm.getInstance().stop(); 
    }

    @Override
    public boolean isFinished() {
        return Arm.getInstance().isBaseZeroed();
    }
}
