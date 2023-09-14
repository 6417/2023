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
import frc.robot.commands.LedsYellowCommand;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.module.Module;
import edu.wpi.first.wpilibj2.command.button.Button;

public class LEDs extends Module  {
    private static LEDs instance;

    private AddressableLED ledsRight;

    private AddressableLEDBuffer ledsBufferRight;

    private static class RGB {
        public int red;
        public int green;
        public int blue;

        public RGB(int gray) {
            red = gray;
            green = gray;
            blue = gray;
        }

        public RGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static RGB back = new RGB(0, 255, 0);
    private static RGB front = new RGB(255, 255, 128);
    private static RGB cube = new RGB(0, 255, 200);
    private static RGB cone = new RGB(75, 255, 0);

    public LEDs() {
        ledsRight = new AddressableLED(Constants.LEDs.portRight);
        ledsRight.setLength(Constants.LEDs.bufferLength);

        ledsBufferRight = new AddressableLEDBuffer(Constants.LEDs.bufferLength);


        ledsRight.start();

        normalLeds();
        setData(); 
    }

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    private void setData() {
        ledsRight.setData(ledsBufferRight);
    }

    @Override
    public void periodic() {
    }



    public void cubeLeds() {
        for (int i = Constants.LEDs.gamePiceOffset; i < Constants.LEDs.bufferLength / 2; i++) {
            ledsBufferRight.setRGB(i, cube.red, cube.green, cube.blue);
            ledsBufferRight.setRGB(Constants.LEDs.bufferLength / 2 + i - Constants.LEDs.gamePiceOffset, cube.red, cube.green, cube.blue);
        }
        setData();
    }

    public void coneLeds() {
        for (int i = Constants.LEDs.gamePiceOffset; i < Constants.LEDs.bufferLength  / 2; i++) {
            ledsBufferRight.setRGB(i, cone.red, cone.green, cone.blue);
            ledsBufferRight.setRGB(Constants.LEDs.bufferLength / 2 + i - Constants.LEDs.gamePiceOffset, cone.red, cone.green, cone.blue);
        }
        setData();
    }

    public void normalLeds() {
        for (int i = 0; i < Constants.LEDs.bufferLength / 2; i++) {
            ledsBufferRight.setRGB(i, front.red, front.green, front.blue);
            ledsBufferRight.setRGB(i+ Constants.LEDs.bufferLength / 2, back.red, back.green, back.blue);
        }
        setData();
    }

    @Override
    public List<Binding> getMappings() {
        Binding cubeLeds = new Binding(Constants.Joysticks.armJoystick, Constants.Gripper.ButtonIds.cubeLeds,
                Button::onTrue, new LedsVioletCommand());
        Binding coneLeds = new Binding(Constants.Joysticks.armJoystick, Constants.Gripper.ButtonIds.coneLeds,
                Button::onTrue, new LedsYellowCommand());
        return List.of(
                cubeLeds, coneLeds);
    }

}
