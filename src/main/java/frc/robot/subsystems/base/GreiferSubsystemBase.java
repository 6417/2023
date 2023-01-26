// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.robot.subsystems.GreiferSubsystem;

public class GreiferSubsystemBase extends SubsystemBase {

  private FridoDoubleSolenoid greiferSolinoid;
  private PneumaticHandler greiferPenumaticHandler;
  private static GreiferSubsystem instance;

  /** Creates a new ExampleSubsystem. */
  public GreiferSubsystemBase() {}

  public void openGreifer(){}

  public void closeGreifer(){}

  public static GreiferSubsystem getInstance(){
    if (instance == null){
      instance = new GreiferSubsystem();
    }
    return instance;
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
