package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.SteerMode;

public class SetSteerMode extends CommandBase {
  private SteerMode mode;

  public SetSteerMode(SteerMode mode) {
    this.mode = mode;
  }

  @Override
  public void initialize() {
    Drive.getInstance().setSteerMode(mode);
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
