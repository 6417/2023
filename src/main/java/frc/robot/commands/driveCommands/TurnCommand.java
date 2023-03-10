// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnCommand extends CommandBase {
  private final DriveBase m_subsystem;
  private double m_rotateByDegrees;
  private Rotation2d m_initRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnCommand(double rotateByDegrees) {
    m_subsystem = Drive.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initRotation = m_subsystem.getPosition().getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() {
    System.out.println( m_subsystem.getPosition()
    .getRotation()
    .minus(m_initRotation)
    .getDegrees());
    //m_subsystem.drive(0.2, 0, Math.signum(m_rotateByDegrees) * 1);
    //tankDrive.arcadeDrive(-xInput, yInput, false)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getPosition()
        .getRotation()
        .minus(m_initRotation)
        .getRadians() > m_rotateByDegrees;

  }
}
