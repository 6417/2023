package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmCalculator;
import frc.robot.subsystems.Arm;

public class HoldAtCurrentPosition extends CommandBase {
    public HoldAtCurrentPosition() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void execute() {
        var torques = Arm.getInstance().getCalculator().calculateTorquesForStall();
        Arm.getInstance().setBaseAmpsLimit(ArmCalculator.torqueToAmps(torques.base));
        Arm.getInstance().setPercentBase(Math.signum(torques.base));
    }
}
