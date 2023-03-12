package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Stop extends CommandBase {
    public Stop() {
        addRequirements(Arm.getInstance());
    } 
    
    @Override
    public void initialize() {
        Arm.getInstance().stop();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
