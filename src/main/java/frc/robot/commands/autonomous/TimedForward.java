// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class TimedForward extends WaitCommand {

  public TimedForward(double secs) {
    super(secs);
    addRequirements(Drive.getInstance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() {
    Drive.getInstance().drive(-0.2, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Drive.getInstance().stop();
  }
}
