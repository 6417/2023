package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class WPIJoystick extends Joystick implements IJoystick {
    public WPIJoystick(IJoystickId port) {
        super(port.getPort());
    }

    @Override
    public Button getButton(IJoystickButtonId id) {
        return new JoystickButton(this, id.getButtonId());
    }
}
