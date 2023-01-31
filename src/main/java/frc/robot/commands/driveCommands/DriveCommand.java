package frc.robot.commands.driveCommands;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.Drive.SteerMode;

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
    Binding driveForward = new Binding(Constants.Joystick.accelerator, LogitechExtreme._8, Button::toggleWhenPressed, new ReverseDrivingDirection(false));
    Binding driveInverted = new Binding(Constants.Joystick.accelerator, LogitechExtreme._12, Button::toggleWhenPressed, new ReverseDrivingDirection(true));

    Binding carMode = new Binding(Constants.Joystick.accelerator, LogitechExtreme._7, Button::onTrue, new SetSteerMode(SteerMode.CARLIKE));
    Binding bidirectionalMode = new Binding(Constants.Joystick.accelerator, LogitechExtreme._9, Button::onTrue, new SetSteerMode(SteerMode.BIDIRECTIONAL));
    JoystickHandler.getInstance().bindAll(List.of(
        driveForward, driveInverted,
        carMode, bidirectionalMode));
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
