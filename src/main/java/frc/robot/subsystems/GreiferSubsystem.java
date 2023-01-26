// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.pneumatics.PneumaticHandler;
import frc.robot.Constants;
import frc.fridowpi.module.Module;

public class GreiferSubsystem extends Module {

  private FridoDoubleSolenoid greiferSolinoid;
  private PneumaticHandler greiferPenumaticHandler;
  private static GreiferSubsystem instance;
  private int status;

  /** Creates a new ExampleSubsystem. */
  public GreiferSubsystem() {
    
    greiferSolinoid = new FridoDoubleSolenoid(Constants.GreiferConstants.greifersolinoidIdlower, Constants.GreiferConstants.greifersolinoidIdhigher);
   

    status = 0;

    //PneumaticHandler.getInstance().enableCompressor();
  }

  @Override
  public void init() {
      // TODO Auto-generated method stub
      greiferSolinoid.init();
      super.init();
  }

  public List<Binding> getMappings() {
        return List.of(
          new Binding(() -> 0, () -> 1, Button::whenPressed, new InstantCommand(() -> this.openGreifer()))
          //new Binding(() -> 0, () -> 2, Button::whenPressed, new InstantCommand(() -> closeGreifer()))
        );
    }

  public void openGreifer(){
    greiferSolinoid.set(Value.kForward);
  }

  public void closeGreifer(){
    greiferSolinoid.set(Value.kReverse);
  }

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
