package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.pneumatics.PneumaticHandler;

public class ToggleCompressor extends CommandBase {
    @Override
    public void initialize() {
        PneumaticHandler.getInstance().enableCompressor();
    } 
    @Override
    public void end(boolean interrupted) {
        PneumaticHandler.getInstance().disableCompressor();
    }
}
