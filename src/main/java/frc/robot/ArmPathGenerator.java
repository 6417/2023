package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Relay.InvalidValueException;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.utils.Vector2;
import frc.robot.commands.arm.PickUpBackward;

public class ArmPathGenerator {
    public static enum RobotPos {
        GRID, LOADING_ZONE, FIELD;
    }

    public static enum RobotOrientation {
        FORWARD, REVERSE;
    }

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

        public Quadrant offset(Vector2 off) {
            if (driveThrough != null) {
                return new Quadrant(from.add(off), to.add(off), driveThrough.add(off));
            } else {
                return new Quadrant(from.add(off), to.add(off), null);
            }
        }

        public static Quadrant[] offset(Quadrant[] quadrants, Vector2 off) {
            return Arrays.stream(quadrants).map((q) -> q.offset(off)).collect(Collectors.toList())
                    .toArray(Quadrant[]::new);
        }
    }

    static final double coneHeight = Constants.Arm.baseArm.length - Constants.Arm.gripperArm.length;
    static final double robotHeight = 0.164;

    static Map<Pair<RobotPos, RobotOrientation>, Quadrant[]> quadrants = new HashMap<>();

    static <T> T[] concat(T[] array1, T[] array2) {
        List<T> resultList = new ArrayList<>(array1.length + array2.length);
        Collections.addAll(resultList, array1);
        Collections.addAll(resultList, array2);

        @SuppressWarnings("unchecked")
        // the type cast is safe as the array1 has the type T[]
        T[] resultArray = (T[]) Array.newInstance(array1.getClass().getComponentType(), 0);
        return resultList.toArray(resultArray);
    }

    static Quadrant[] quadrantsField = new Quadrant[] {
            new Quadrant(new Vector2(0.26, -0.05), new Vector2(1.44, 0.05), null),
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(1.44, 1.78), new Vector2(0.29, coneHeight + 0.01)),
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(-0.56, 1.78),
                    new Vector2(0, coneHeight)),
            new Quadrant(new Vector2(-0.56, 0.05), new Vector2(-1.74, 1.78), new Vector2(-0.75, coneHeight + 0.01)),
            new Quadrant(new Vector2(-0.56, -0.05), new Vector2(-1.74, 0.05), null)
    };

    static {

        Quadrant[] quadrantsLoadingZone = new Quadrant[] {
                new Quadrant(new Vector2(0, -0.5), new Vector2(0.1, 1.78), new Vector2(-0.05, 1.2)),
                new Quadrant(new Vector2(0, 1.0 - robotHeight), new Vector2(0.3, 1.78), null),
        };

        Quadrant[] quadrantsGrid = new Quadrant[] {
                new Quadrant(new Vector2(0.0, 0.25), new Vector2(0.3, 1.74), new Vector2(0.25, 1.05 + coneHeight)),
                new Quadrant(new Vector2(0.30, 1.05), new Vector2(0.75, 1.74), new Vector2(0.6, 1.35)),
                new Quadrant(new Vector2(0.75, 1.3), new Vector2(1.1, 1.74), null),
        };

        Vector2 forwardOffset = new Vector2(0.46, 0.0);
        Vector2 reverseOffset = new Vector2(-0.60, 0.0);

        Quadrant[] fieldForward = new Quadrant[] {
                quadrantsField[4],
                quadrantsField[3],
                quadrantsField[2],
                quadrantsField[1],
        };

        Quadrant[] fieldReverse = new Quadrant[] {
                quadrantsField[0],
                quadrantsField[1],
                quadrantsField[2],
                quadrantsField[3],
        };

        quadrants.put(new Pair<>(RobotPos.FIELD, RobotOrientation.FORWARD), quadrantsField);
        quadrants.put(new Pair<>(RobotPos.FIELD, RobotOrientation.REVERSE), quadrantsField);

        quadrants.put(new Pair<>(RobotPos.LOADING_ZONE, RobotOrientation.FORWARD),
                concat(Quadrant.offset(quadrantsLoadingZone, forwardOffset), fieldForward));
        quadrants.put(new Pair<>(RobotPos.LOADING_ZONE, RobotOrientation.REVERSE),
                concat(Quadrant.offset(quadrantsLoadingZone, reverseOffset), fieldReverse));

        quadrants.put(new Pair<>(RobotPos.GRID, RobotOrientation.FORWARD),
                concat(Quadrant.offset(quadrantsGrid, forwardOffset), fieldForward));
        quadrants.put(new Pair<>(RobotPos.GRID, RobotOrientation.REVERSE),
                concat(Quadrant.offset(quadrantsGrid, reverseOffset), fieldReverse));
    }

    private int getQuadrantIndexOfPoint(Vector2 point, RobotPos pos, RobotOrientation orientation)
            throws InvalidValueException {
        Quadrant[] quadrants = ArmPathGenerator.quadrants.get(new Pair<>(pos, orientation));
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

    public boolean isValidInField(Vector2 pos) {
        for (var q : quadrantsField) {
            if (q.contains(pos)) {
                return true;
            }
        }
        return false;
    }

    public Vector2[] pathTo(Vector2 target, RobotPos pos, RobotOrientation orientation) {
        try {
            int indexPos = getQuadrantIndexOfPoint(ArmKinematics.anglesToPos(baseAngle.get(), jointAngle.get()), pos,
                    orientation);
            int indexTarget = getQuadrantIndexOfPoint(target, pos, orientation);

            if (Math.abs(indexPos - indexTarget) <= 1) {
                return new Vector2[] { target };
            }

            int sign = signum(indexTarget - indexPos);
            Vector2[] path = new Vector2[Math.abs(indexTarget - indexPos) + 1];

            int i = 0;
            for (int quad = indexPos + sign; quad != indexTarget - sign; quad += sign) {
                path[i] = quadrants.get(new Pair<>(pos, orientation))[quad].driveThrough;
                i++;
            }

            path[path.length - 1] = target;
            return path;
        } catch (InvalidValueException e) {
            return new Vector2[] {};
        }
    }
}
