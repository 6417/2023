package frc.llib;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.command.Command;

public class ShortCuts {

    public void schedule(Command cmd) {
        CommandScheduler.getInstance().schedule(cmd);
    }
}
