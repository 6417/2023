package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Relay.InvalidValueException;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.utils.Vector2;

public class ArmPathGenerator {
    private Supplier<Double> baseAngle;
    private Supplier<Double> jointAngle;

    private static class Quadrant {
        public Vector2 from;
        public Vector2 to;
        public Vector2 driveThrough;

        public Quadrant(Vector2 from, Vector2 to, Vector2 driveThrough) {
            this.from = from;
            this.to = to;
            this.driveThrough = driveThrough;
        }

        private boolean isBetween(double val, double rangeA, double rangeB) {
            double min = Math.min(rangeA, rangeB);
            double max = Math.max(rangeA, rangeB);

            return min <= val && val <= max;
        }

        public boolean contains(Vector2 point) {
            return isBetween(point.x, from.x, to.x) && isBetween(point.y, from.y, to.y);
        }
    }

    static final double coneHeight = Constants.Arm.baseArm.length - Constants.Arm.gripperArm.length;
    Quadrant[] quadrants = new Quadrant[] {
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(1.44, -0.164), null),
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(1.44, 1.78), new Vector2(0.42, coneHeight)),
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(-0.56, 1.78),
                    new Vector2(0, coneHeight)),
            new Quadrant(new Vector2(-0.56, 0.05), new Vector2(-1.74, 178), new Vector2(-72, coneHeight)),
            new Quadrant(new Vector2(-0.56, 0.05), new Vector2(-1.74, -0.164), null)
    };

    private int getQuadrantIndexOfPoint(Vector2 point) throws InvalidValueException {
        for (int i = 0; i < quadrants.length; i++) {
            if (quadrants[i].contains(point)) {
                return i;
            }
        }

        throw new InvalidValueException(null);
    }

    private int signum(int num) {
        if (num < 0) {
            return -1;
        } else if (num > 0) {
            return 1;
        } else {
            return 0;
        }
    }

    public Vector2[] pathTo(Vector2 target) {
        try {
            int indexPos = getQuadrantIndexOfPoint(ArmKinematics.anglesToPos(baseAngle.get(), jointAngle.get()));
            int indexTarget = getQuadrantIndexOfPoint(target);

            if (Math.abs(indexPos - indexTarget) <= 1) {
                return new Vector2[] { target };
            }

            int sign = signum(indexTarget - indexPos);
            Vector2[] path = new Vector2[Math.abs(indexTarget - indexPos) + 1];

            int i = 0;
            for (int quad = indexPos + sign; quad != indexTarget - sign; quad += sign) {
                path[i] = quadrants[quad].driveThrough;
                i++;
            }

            path[path.length - 1] = target;
            return path;
        } catch (InvalidValueException e) {
            return new Vector2[] {};
        }
    }
}
