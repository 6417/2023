package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class InstantCommandRunWhenDisabled extends InstantCommand {
    public InstantCommandRunWhenDisabled(Runnable toRun, Subsystem... requirements)  {
        super(toRun, requirements);
    }
}
