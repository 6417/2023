// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.LedsVioletCommand;
import frc.fridowpi.joystick.Binding;
import edu.wpi.first.wpilibj2.command.button.Button;

public class LEDs extends SubsystemBase {
  private static LEDs instance;

  private AddressableLED ledsFrontRight;
  private AddressableLED ledsFrontLeft;
  private AddressableLED ledsBackRight;
  private AddressableLED ledsBackLeft;

  private AddressableLEDBuffer ledsBufferFrontRight;
  private AddressableLEDBuffer ledsBufferFrontLeft;
  private AddressableLEDBuffer ledsBufferBackRight;
  private AddressableLEDBuffer ledsBufferBackLeft;

 
  public LEDs() {
    ledsFrontRight = new AddressableLED(Constants.LEDs.portFrontRight);
    ledsFrontLeft = new AddressableLED(Constants.LEDs.portFrontLeft);
    ledsBackRight = new AddressableLED(Constants.LEDs.portBackRight);
    ledsBackRight = new AddressableLED(Constants.LEDs.portBackLeft);

    ledsBufferFrontRight = new AddressableLEDBuffer(Constants.LEDs.bufferFrontRight);
    ledsBufferFrontLeft = new AddressableLEDBuffer(Constants.LEDs.bufferFrontLeft);
    ledsBufferBackRight = new AddressableLEDBuffer(Constants.LEDs.bufferBackRight);
    ledsBufferBackLeft = new AddressableLEDBuffer(Constants.LEDs.bufferBackLeft);

    ledsFrontRight.setLength(ledsBufferFrontRight.getLength());
    ledsFrontLeft.setLength(ledsBufferFrontLeft.getLength());
    ledsBackRight.setLength(ledsBufferBackRight.getLength());
    ledsBackLeft.setLength(ledsBufferBackLeft.getLength());

    ledsFrontRight.setData(ledsBufferFrontRight);
    ledsFrontLeft.setData(ledsBufferFrontLeft);
    ledsBackRight.setData(ledsBufferBackRight);
    ledsBackLeft.setData(ledsBufferBackLeft);

    ledsFrontRight.start();
    ledsFrontLeft.start();
    ledsBackRight.start();
    ledsBackLeft.start();

    normalLeds();
  }
  public static LEDs getInstance() {
    if (instance != null) {
        return instance;
    }
    return instance;
}

public void cubeLeds() {
  for(int i = 30; i<60; i++){
    ledsBufferBackLeft.setRGB(i, 140, 0, 220);
    ledsBufferFrontLeft.setRGB(i, 140, 0, 220);
    ledsBufferBackRight.setRGB(i, 140, 0, 220);
    ledsBufferFrontRight.setRGB(i, 140, 0, 220);
  }
}
public void coneLeds() {
  for(int i = 30; i<60; i++){
    ledsBufferBackLeft.setRGB(i, 255, 180, 0);
    ledsBufferFrontLeft.setRGB(i, 255, 180, 0);
    ledsBufferBackRight.setRGB(i, 255, 180, 0);
    ledsBufferFrontRight.setRGB(i, 255, 180, 0);
  }
}
public void normalLeds() {
  for(int i = 0; i<60; i++){
    ledsBufferBackLeft.setRGB(i, 255, 0, 0);
    ledsBufferFrontLeft.setRGB(i, 0, 180, 0);
    ledsBufferBackRight.setRGB(i, 255, 0, 0);
    ledsBufferFrontRight.setRGB(i, 0, 180, 0);
  }
}

public List<Binding> getMappings() {
  Binding cubeLeds = new Binding(Constants.Joystick.accelerator, Constants.Drive.ButtonIds.cubeLeds, Button::toggleOnTrue, new LedsVioletCommand());
  Binding coneLeds = new Binding(Constants.Joystick.accelerator, Constants.Drive.ButtonIds.coneLeds, Button::toggleOnTrue, new LedsYellowCommand()); 
  return List.of(
  cubeLeds, coneLeds
  );
}

}
