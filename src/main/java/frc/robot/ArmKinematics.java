package frc.robot;

import edu.wpi.first.math.Pair;
import frc.fridowpi.utils.Matrix2;
import frc.fridowpi.utils.Vector2;

/**
 * ArmKinematics
 */
public class ArmKinematics {
    public static class Values<T> {
        public T base;
        public T joint;

        public Values(T base, T joint) {
            this.base = base;
            this.joint = joint;
        }
    }

    private static final double gripperArmLen = Constants.Arm.gripperArm.length;
    private static final double baseArmLen = Constants.Arm.baseArm.length;

    public static Vector2 anglesToPos(double base, double joint) {
        joint *= -1;
        final Vector2 vecBase = new Vector2(baseArmLen, 0);
        final Vector2 vecGripper = new Vector2(-gripperArmLen, 0.0);

        return Matrix2.rot(base).vmul(vecBase).add(Matrix2.rot(base).mul(Matrix2.rot(joint)).vmul(vecGripper));
    }

    public static Pair<Values<Double>, Values<Double>> posToAngles(Vector2 pos) {
        double c = pos.magnitude();
        double phi = Math
                .acos((Math.pow(baseArmLen, 2) - Math.pow(gripperArmLen, 2) + Math.pow(c, 2)) / (2 * baseArmLen * c));

        double theta = Math.atan2(pos.y, pos.x);

        double alpha1 = -phi;
        double alpha2 = +phi;

        Vector2 u1 = Matrix2.rot(alpha1).vmul(new Vector2(baseArmLen, 0));
        Vector2 u2 = Matrix2.rot(alpha2).vmul(new Vector2(baseArmLen, 0));

        Vector2 v1 = pos.minus(u1);
        Vector2 v2 = pos.minus(u2);

        double beta1 = Math.acos(u1.dot(v1) / (baseArmLen * gripperArmLen));
        double beta2 = Math.acos(u2.dot(v2) / (baseArmLen * gripperArmLen));

        return new Pair<>(new Values<>(alpha1, -beta1), new Values<>(alpha2, -beta2));
    }
}
