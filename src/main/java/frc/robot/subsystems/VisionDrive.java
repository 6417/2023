// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.driveCommands.AimeLineCommand;
import frc.robot.commands.driveCommands.TurnCommand;
import frc.robot.subsystems.drive.Drive;

public class VisionDrive extends SubsystemBase {

  public static VisionDrive instance;

  public double getTagdrive(int aimetag){
    double tagPose = 0;
    if (aimetag == 0){
        tagPose = -0.53; 
    }else if (aimetag == 1){
        tagPose = 0;
    }else{
        tagPose = 0.53;
    }
    return tagPose;
  }
  
  public double getGrid(){
      double pos = Drive.getInstance().getPosition().getTranslation().getY();
      double min = 10;
      double[] pos2 = {0.95, 2.75, 4.5};
      for (int i = 0; i < 3; i++){
          if (Math.abs(pos2[i]-pos) < min){
              min = Math.abs(pos2[i]-pos);
          }
      }
      return min;
  }

  private SequentialCommandGroup getCommandGroup(int x){
    return new SequentialCommandGroup(
      new TurnCommand(Drive.getInstance().getPosition().getRotation().getDegrees()), //turn to line
      new AimeLineCommand(new Pose2d(Drive.getInstance().getPosition().getTranslation().getX()-2.29,0, new Rotation2d(0))), //drive to line
      new TurnCommand(Math.PI/2), // turn neinti degrees
      new AimeLineCommand(new Pose2d(Drive.getInstance().getPosition().getTranslation().getX()-getGrid()-getTagdrive(x), 1, new Rotation2d(0))), //drive to zielpos
      new TurnCommand(Math.PI/2), // turn neinti gegrees
      new AimeLineCommand(new Pose2d(0.9, 0, new Rotation2d(0)))// drive to station
    );
  }

  public void execute(int x){
    CommandScheduler.getInstance().schedule(getCommandGroup(x));
  }

  public static VisionDrive getInstance(){
    if (instance == null){
      instance = new VisionDrive();
    }
    return instance;
  }

  /** Creates a new ExampleSubsystem. */
  public VisionDrive() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}