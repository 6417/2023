// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import frc.robot.subsystems.VisionDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class VisionCommand extends CommandBase {
  private final DriveBase m_subsystem;
  int x;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionCommand(int x) {
    m_subsystem = Drive.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionDrive.getInstance().execute(x);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
