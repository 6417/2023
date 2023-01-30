package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.joystick.JoystickHandler;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;

public class DriveCommand extends CommandBase {
  private final DriveBase m_subsystem;


  public DriveCommand() {
    m_subsystem = Drive.getInstance();
    // Declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    JoystickHandler.getInstance()
        .setupJoysticks(List.of(Constants.Joystick.accelerator, Constants.Joystick.steeringWheel));
  }

  // Called every time the scheduler runs while the command is scheduled.
  // (Aka .periodic())
  @Override
  public void execute() {

    // Get input
    double joystick_y_input = JoystickHandler
      .getInstance()
      .getJoystick(Constants.Joystick.accelerator)
      .getY();
    double steer_input = JoystickHandler
      .getInstance()
      .getJoystick(Constants.Joystick.steeringWheel)
      .getX();

    // Call Drive::drive with the input
    m_subsystem.drive(joystick_y_input, steer_input);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
