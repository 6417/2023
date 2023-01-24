package frc.fridowpi.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ICommand extends Command {
    void requires(Subsystem... requirements);
}
