package frc.robot.commands.driveCommands;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BreakCommand extends CommandBase {

  public BreakCommand() {
    addRequirements(Drive.getInstance());
  }
  
  @Override
  public void initialize() {
    Drive.getInstance().triggerBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drive.getInstance().releaseBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
