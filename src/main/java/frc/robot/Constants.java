// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.Logitech;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Joysticks {
    public static final IJoystickId drive = () -> 0;
  }
  public static class Gripper {
    public static class ButtonIds {
      public static final IJoystickButtonId openClose = Logitech.a;

    }
    public static final boolean enabled = true;
    public static final int gripperSolenoidIdLower = 0;
    public static final int gripperSolenoidIdHigher = 1;
    public static final int gripperCompressorId = 0;
    public static final PneumaticsModuleType type = PneumaticsModuleType.CTREPCM;
  }
}
