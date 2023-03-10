// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AimeLineCommand extends CommandBase {
  private final DriveBase m_subsystem;
  private Pose2d m_position;
  private Translation2d m_initPos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimeLineCommand(Pose2d pose2d) {
    m_subsystem = Drive.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initPos = m_subsystem.getPosition().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() {
    m_subsystem.drive(0.2, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  private double getd(Translation2d pos){
    return Math.sqrt(pos.getX()*pos.getX() + pos.getY()*pos.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getd(m_subsystem.getPosition()
        .getTranslation()
        .minus(m_initPos)) > getd(new Translation2d(0.1, 0.1));
  }
}
