package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

/** An example command that uses an example subsystem. */
public class ReverseDrivingDirection extends CommandBase {

  public ReverseDrivingDirection(boolean reverse) {
    Drive.getInstance().reverseDrivingDirection(reverse);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
