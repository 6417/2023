package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

/** An example command that uses an example subsystem. */
public class ReverseDrivingDirection extends CommandBase {

  private boolean reverse;

  public ReverseDrivingDirection(boolean reverse) {
    this.reverse = reverse;
    System.out.println(reverse);
  }

  @Override
  public void initialize() {
    Drive.getInstance().reverseDrivingDirection(reverse);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
