package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;

public class IndividualArmManualControl extends ParallelCommandGroup {
    public IndividualArmManualControl() {
        addCommands(new JointManualControl(), new BaseManualControl());
        addRequirements(Arm.getInstance());
    } 
}
