package frc.fridowpi.joystick;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface IJoystick {
    Button getButton(IJoystickButtonId id);

    double getX();

    double getY();

    double getZ();

    double getTwist();

    double getThrottle();

    boolean getTrigger();

    boolean getTriggerPressed();

    boolean getTriggerReleased();

    boolean getTop();

    boolean getTopPressed();

    boolean getTopReleased();

    double getMagnitude();

    double getDirectionRadians();

    double getDirectionDegrees();
}
