package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

import java.util.function.BiConsumer;

public class Binding {
    public BiConsumer<Button, Command> action;
    public IJoystickButtonId buttonId;
    public IJoystickId joystickId;
    public Command command;

    public Binding(IJoystickId joystickId, IJoystickButtonId buttonId, BiConsumer<Button, Command> action, Command command) {
        this.action = action;
        this.buttonId = buttonId;
        this.joystickId = joystickId;
        this.command = command;
    }
}
