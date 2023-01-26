// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.robot.Constants;
import frc.robot.Constants.Gripper;
import frc.robot.commands.GripperCommand;
import frc.robot.subsystems.base.GripperSubsystemBase;


public class GripperSubsystem extends GripperSubsystemBase {

  private FridoDoubleSolenoid gripperSolenoid;
  private static GripperSubsystemBase instance;

  public GripperSubsystem() {
    gripperSolenoid = new FridoDoubleSolenoid(Constants.Gripper.gripperSolenoidIdLower, Constants.Gripper.gripperSolenoidIdHigher);
  }

  @Override
  public void init() {
      // TODO Auto-generated method stub
      gripperSolenoid.init();
      super.init();
  }

  public List<Binding> getMappings() {
        return List.of(
          new Binding(Constants.Joysticks.drive, Constants.Gripper.ButtonIds.openClose, Button::toggleOnTrue, new GripperCommand())
          );
    }

  public void openGripper() {
    gripperSolenoid.set(Value.kForward);
  }

  public void closeGripper() {
    gripperSolenoid.set(Value.kReverse);
  }

  public static GripperSubsystemBase getInstance() {
    if (instance == null){
      if(Constants.Gripper.enabled){
        instance = new GripperSubsystem();
      }
      else{
        instance = new GripperSubsystemBase();
      }
    }
    return instance;
  }
}
