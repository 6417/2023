package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ResetBaseEncodersOnLimitSwitch extends CommandBase {
    @Override
    public void execute() {
        if (Arm.getInstance().getBaseLimitSwitchFwd())  {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseEncoderPosFwdLimitSwitch);
        }
        
        if (Arm.getInstance().getBaseLimitSwitchRev())  {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseEncoderPosRevLimitSwitch);
        }
    } 
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
