package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.Logitech;

public final class Constants {
  public static class Joysticks {
    public static final IJoystickId drive = () -> 0;
  }

  public static class Gripper {
    public static final boolean enabled = true;

    public static class ButtonIds {
      public static final IJoystickButtonId openClose = Logitech.a;
    }
    
    public static class DoubleSolenoid {
      public static final int idLower = 0;
      public static final int idHigher = 1;
      public static final int compressorId = 0;
      public static final PneumaticsModuleType type = PneumaticsModuleType.CTREPCM;
      public static final Value gripperOpen = Value.kForward;
      public static final Value gripperClosed = Value.kReverse;
    }
  }
}