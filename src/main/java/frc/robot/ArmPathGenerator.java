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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.fridowpi.utils.Matrix2;
import frc.fridowpi.utils.Vector2;
import frc.robot.commands.arm.GotoPosNoChecks;
import frc.robot.commands.arm.PickUpBackward;
import frc.robot.subsystems.Arm;

public class ArmPathGenerator {
    public static enum RobotPos {
        GRID, LOADING_ZONE, FIELD;
    }

    public static enum RobotOrientation {
        FORWARD, REVERSE;
    }

    private Supplier<Double> baseAngle;
    private Supplier<Double> jointAngle;

    private static class Pair<A, B> extends edu.wpi.first.math.Pair<A, B> {
        public Pair(A first, B second) {
            super(first, second);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof Pair<?, ?>) {
                var otherFirst = ((Pair<?, ?>) obj).getFirst();
                var otherSecond = ((Pair<?, ?>) obj).getSecond();

                return getFirst().equals(otherFirst) && getSecond().equals(otherSecond);
            }
            return super.equals(obj);
        }
    }

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

        public static Quadrant[] flip(Quadrant[] quadrants) {
            return Arrays.stream(quadrants).map((q) -> q.flip()).collect(Collectors.toList())
                    .toArray(Quadrant[]::new);
        }

        public Quadrant flip() {
            Matrix2 matFlip = new Matrix2(new double[][] {
                    { -1, 0 },
                    { 0, 1 }
            });
            if (driveThrough != null) {
                return new Quadrant(matFlip.vmul(from), matFlip.vmul(to), matFlip.vmul(driveThrough));
            } else {
                return new Quadrant(matFlip.vmul(from), matFlip.vmul(to), null);
            }
        }
    }

    public static final double coneHeight = Constants.Arm.baseArm.length - Constants.Arm.gripperArm.length;
    static final double robotHeight = 0.164;

    static Map<RobotPos, Map<RobotOrientation, Quadrant[]>> quadrants = new HashMap<>();

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
            new Quadrant(new Vector2(0.26, 0.05), new Vector2(-0.61, 1.78),
                    new Vector2(0, coneHeight)),
            new Quadrant(new Vector2(-0.56, 0.05), new Vector2(-1.74, 1.78), new Vector2(-0.75, coneHeight + 0.01)),
            new Quadrant(new Vector2(-0.56, -0.05), new Vector2(-1.74, 0.05), null)
    };

    public static Vector2 forwardOffset = new Vector2(0.26, 0.0);
    public static Vector2 reverseOffset = new Vector2(0.61, 0.0);

    static {

        // ORDER MATTERS!
        Quadrant[] quadrantsLoadingZone = new Quadrant[] {
                new Quadrant(new Vector2(0, 1.0 - robotHeight), new Vector2(0.3, 1.78), null),
                new Quadrant(new Vector2(-0.05, -0.05), new Vector2(0.0, 1.78), new Vector2(-0.05, 1.2)),
        };

        Quadrant[] quadrantsGrid = new Quadrant[] {
                new Quadrant(new Vector2(0.75, 1.07), new Vector2(1.44, 1.74), null),
                new Quadrant(new Vector2(0.30, 0.78), new Vector2(0.75, 1.74), new Vector2(0.6, 1.07 + coneHeight)),
                new Quadrant(new Vector2(0.0, 0.20), new Vector2(0.29, 1.74), new Vector2(0.25, 0.78 + coneHeight)),
        };


        Quadrant[] fieldForward = new Quadrant[] {
                quadrantsField[2],
                quadrantsField[3],
                quadrantsField[4],
        };

        Quadrant[] fieldReverse = new Quadrant[] {
                quadrantsField[2],
                quadrantsField[1],
                quadrantsField[0],
        };

        Map<RobotOrientation, Quadrant[]> quadrantsFieldMap = new HashMap<>();
        quadrantsFieldMap.put(RobotOrientation.FORWARD, quadrantsField);
        quadrantsFieldMap.put(RobotOrientation.REVERSE, quadrantsField);
        quadrants.put(RobotPos.FIELD, quadrantsFieldMap);

        Map<RobotOrientation, Quadrant[]> quadrantsLoadingZoneMap = new HashMap<>();
        quadrantsLoadingZoneMap.put(RobotOrientation.FORWARD,
                concat(Quadrant.offset(quadrantsLoadingZone, forwardOffset), fieldForward));
        quadrantsLoadingZoneMap.put(RobotOrientation.REVERSE,
                concat(Quadrant.flip(Quadrant.offset(quadrantsLoadingZone, reverseOffset)), fieldReverse));
        quadrants.put(RobotPos.LOADING_ZONE, quadrantsLoadingZoneMap);

        Map<RobotOrientation, Quadrant[]> quadrantsGridMap = new HashMap<>();
        quadrantsGridMap.put(RobotOrientation.FORWARD,
                concat(Quadrant.offset(quadrantsGrid, forwardOffset), fieldForward));
        quadrantsGridMap.put(RobotOrientation.REVERSE,
                concat(Quadrant.flip(Quadrant.offset(quadrantsGrid, reverseOffset)), fieldReverse));
        quadrants.put(RobotPos.GRID, quadrantsGridMap);
    }
    
    public int getQuadrantIndexOfPoint(Vector2 point, RobotPos pos, RobotOrientation orientation)
            throws InvalidValueException {
        Quadrant[] quadrants = ArmPathGenerator.quadrants.get(pos).get(orientation);
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

    public static SequentialCommandGroup toCommand(Vector2[] path) {
        SequentialCommandGroup cmd = new SequentialCommandGroup();

        for (var p : path) {
            cmd.addCommands(new GotoPosNoChecks(p));
        }
        cmd.addRequirements(Arm.getInstance());
        return cmd; 
    }

    public Vector2[] pathTo(Vector2 target, RobotPos pos, RobotOrientation orientation) {
        try {
            int indexPos = getQuadrantIndexOfPoint(Arm.getInstance().getPos(), pos,
                    orientation);
            int indexTarget = getQuadrantIndexOfPoint(target, pos, orientation);

            if (Math.abs(indexPos - indexTarget) <= 1) {
                return new Vector2[] { target };
            }

            int sign = signum(indexTarget - indexPos);
            Vector2[] path = new Vector2[Math.abs(indexTarget - indexPos)];

            System.out.println("path length: " + (Math.abs(indexTarget - indexPos)));
            int i = 0;
            for (int quad = indexPos + sign; quad != indexTarget; quad += sign) {
                path[i] = quadrants.get(pos).get(orientation)[quad].driveThrough;
                System.out.printf("quad: %d, pos: %s, orientation: %s, driveThrough: %s\n", quad, pos.toString(), orientation.toString(), path[i]);
                i++;
            }

            path[path.length - 1] = target;
            return path;
        } catch (InvalidValueException e) {
            return new Vector2[] {};
        }
    }
}
