package frc.fridowpi.joystick;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.Map;

public interface JoystickBindable {
    List<Binding> getMappings();
}
