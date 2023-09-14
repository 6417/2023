package frc.robot.commands;

import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LEDs.LEDs;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GripperCommand extends CommandBase {

  public GripperCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(GripperSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GripperSubsystem.getInstance().openGripper();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GripperSubsystem.getInstance().closeGripper();
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> {}, LEDs.getInstance()));
    LEDs.getInstance().normalLeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
