package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmPathGenerator.RobotOrientation;
import frc.robot.ArmPathGenerator.RobotPos;

public class ArmPosJoystick extends WPIJoystick {
    public static enum POV {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        public final int angle;

        private POV(int angle) {
            this.angle = angle;
        }

    }

    private static final int idCounterStart = 1000;
    private static int idCounter = idCounterStart;
    private static final IJoystickButtonId forward = Logitech.rt;
    private static final IJoystickButtonId reverse = Logitech.lt;

    public static enum Ids implements IJoystickButtonId {
        GRID_FORWARD_BOTTOM(POV.DOWN, forward, null,
                new Vector2(0.927, 0.985).minus(ArmPathGenerator.reverseOffset).minus(ArmPathGenerator.forwardOffset),
                RobotPos.GRID, RobotOrientation.FORWARD),
        GRID_FORWARD_MIDDLE(POV.LEFT, forward, null, new Vector2(0.927, 0.985), RobotPos.GRID,
                RobotOrientation.FORWARD),
        GRID_FORWARD_TOP(POV.UP, forward, null, new Vector2(1.34, 1.349), RobotPos.GRID, RobotOrientation.FORWARD),
        GRID_REVERSE_BOTTOM(POV.DOWN, reverse, null, new Vector2(-0.971, 0.26), RobotPos.GRID,
                RobotOrientation.REVERSE),
        GRID_REVERSE_MIDDLE(POV.LEFT, reverse, null, new Vector2(-1.155, 1.118), RobotPos.GRID,
                RobotOrientation.REVERSE),
        HOME(null, Logitech.b, null, new Vector2(0.0, ArmPathGenerator.coneHeight), RobotPos.FIELD,
                RobotOrientation.FORWARD),
        LOADING_ZONE_FORWARD_CUBE(null, forward, Logitech.x, new Vector2(0.506, 0.875), RobotPos.LOADING_ZONE,
                RobotOrientation.FORWARD),
        LOADING_ZONE_FORWARD_CONE(null, forward, Logitech.y, new Vector2(0.454, 0.993), RobotPos.LOADING_ZONE,
                RobotOrientation.FORWARD),
        LOADING_ZONE_REVERSE_CUBE(null, reverse, Logitech.x, new Vector2(-0.862, 0.992), RobotPos.LOADING_ZONE,
                RobotOrientation.REVERSE),
        LOADING_ZONE_REVERSE_CONE(null, reverse, Logitech.y, new Vector2(-0.80, 1.065), RobotPos.LOADING_ZONE,
                RobotOrientation.REVERSE),
        PICKUP_FORWARD(POV.RIGHT, forward, null, new Vector2(0.465, 0.208), RobotPos.FIELD, RobotOrientation.FORWARD),
        PICKUP_REVERSE(POV.RIGHT, reverse, null,
                new Vector2(0.465, 0.208).minus(ArmPathGenerator.reverseOffset).minus(ArmPathGenerator.forwardOffset),
                RobotPos.FIELD, RobotOrientation.REVERSE);

        private final int id;
        public final POV pov;
        public final IJoystickButtonId button1;
        public final IJoystickButtonId button2;
        public final Vector2 target;
        public final ArmPathGenerator.RobotPos pos;
        public final ArmPathGenerator.RobotOrientation orientation;

        private Ids(POV pov, IJoystickButtonId button1, IJoystickButtonId button2, Vector2 targetPos,
                ArmPathGenerator.RobotPos pos,
                ArmPathGenerator.RobotOrientation orientation) {
            id = idCounter;
            idCounter++;

            this.pov = pov;
            this.button1 = button1;
            this.button2 = button2;
            this.target = targetPos;
            this.pos = pos;
            this.orientation = orientation;
        }

        public Button toButtonOnJoystick(WPIJoystick j) {
            final BooleanSupplier button1Pressed;
            if (button1 != null) {
                button1Pressed = () -> j.getButton(button1).getAsBoolean();
            } else {
                button1Pressed = () -> true;
            }

            final BooleanSupplier button2Pressed;
            if (button2 != null) {
                button2Pressed = () -> j.getButton(button2).getAsBoolean();
            } else {
                button2Pressed = () -> true;
            }

            final BooleanSupplier povPressed;
            if (pov != null) {
                povPressed = () -> j.getPOV() == pov.angle;
            } else {
                povPressed = () -> true;
            }
            return new Button((BooleanSupplier) () -> povPressed.getAsBoolean() && button1Pressed.getAsBoolean()
                    && button2Pressed.getAsBoolean());
        }

        @Override
        public int getButtonId() {
            return id;
        }
    }

    public ArmPosJoystick(IJoystickId port) {
        super(port);
    }

    @Override
    public Button getButton(IJoystickButtonId id) {
        if (id.getButtonId() > idCounterStart) {
            return ((Ids) id).toButtonOnJoystick(this);
        }
        return getButton(id);
    }
}
