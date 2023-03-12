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
    public static final Vector2 homePos = ArmKinematics.anglesToPos(Math.PI / 2, 0).add(new Vector2(0, 0.000001));

    public static enum Ids implements IJoystickButtonId {
        GRID_FORWARD_BOTTOM(POV.DOWN, forward, null,
                new Vector2(0.54, 0.26),
                RobotPos.FIELD, RobotOrientation.FORWARD),
        GRID_FORWARD_MIDDLE(POV.LEFT, forward, null, new Vector2(0.968, 0.934), RobotPos.GRID,
                RobotOrientation.FORWARD),
        GRID_FORWARD_TOP(POV.UP, forward, null, new Vector2(1.214, 1.364), RobotPos.GRID, RobotOrientation.FORWARD),
        GRID_REVERSE_BOTTOM(POV.DOWN, reverse, null, new Vector2(-0.89, 0.26), RobotPos.FIELD,
                RobotOrientation.REVERSE),
        GRID_REVERSE_MIDDLE(POV.LEFT, reverse, null, new Vector2(-1.214, 1.156), RobotPos.GRID,
                RobotOrientation.REVERSE),
        HOME(null, Logitech.b, null, homePos, RobotPos.FIELD,
                RobotOrientation.FORWARD),
        LOADING_ZONE_FORWARD_CUBE(null, reverse, Logitech.x, new Vector2(0.467, 0.89), RobotPos.LOADING_ZONE,
                RobotOrientation.FORWARD),
        LOADING_ZONE_FORWARD_CONE(null, reverse, Logitech.y, new Vector2(0.50, 0.95), RobotPos.LOADING_ZONE,
                RobotOrientation.FORWARD),
        LOADING_ZONE_REVERSE_CUBE(null, forward, Logitech.x, new Vector2(-0.772, 1.08), RobotPos.LOADING_ZONE,
                RobotOrientation.REVERSE),
        LOADING_ZONE_REVERSE_CONE(null, forward, Logitech.y, new Vector2(-0.799, 1.128), RobotPos.LOADING_ZONE,
                RobotOrientation.REVERSE),
        PICKUP_FORWARD(POV.RIGHT, reverse, null, new Vector2(0.465, 0.208), RobotPos.FIELD, RobotOrientation.FORWARD),
        PICKUP_REVERSE(POV.RIGHT, forward, null, new Vector2(-0.70, 0.16), RobotPos.FIELD,
                RobotOrientation.REVERSE);

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
            System.out.println("initializing pos joystick ids");
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
        if (id.getButtonId() >= idCounterStart) {
            return ((Ids) id).toButtonOnJoystick(this);
        }
        return super.getButton(id);
    }
}
