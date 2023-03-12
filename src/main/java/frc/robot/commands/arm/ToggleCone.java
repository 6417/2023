package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmModel.Cargo;
import frc.robot.subsystems.Arm;

public class ToggleCone extends CommandBase {
    @Override
    public void initialize() {
        if (Arm.getInstance().getModel().getCargo() != Cargo.Cone)
            Arm.getInstance().getModel().updateCargo(Cargo.Cone);
        else
            Arm.getInstance().getModel().updateCargo(Cargo.None);
    }

    @Override
    public void end(boolean interrupted) {
        if (Arm.getInstance().getModel().getCargo() != Cargo.Cone)
            Arm.getInstance().getModel().updateCargo(Cargo.Cone);
        else
            Arm.getInstance().getModel().updateCargo(Cargo.None);
    }
}
