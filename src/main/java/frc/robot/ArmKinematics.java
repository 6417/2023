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

    private static double sq(double v) {
        return v * v;
    }

    public static Vector2 anglesToPos(double base, double joint) {
        // double a = Math.sqrt(sq(baseArmLen) + sq(gripperArmLen) - 2 * baseArmLen *
        // gripperArmLen * Math.cos(-joint));
        // double phi = Math.acos((sq(a) + sq(baseArmLen) - sq(gripperArmLen)) / (2 *
        // baseArmLen * a));
        // double theta = base - phi;

        // return new Vector2(Math.cos(theta) * a, Math.sin(theta) * a);
        base = Math.PI - base; 
        Vector2 u = new Vector2(baseArmLen, 0);
        Vector2 v = new Vector2(-gripperArmLen, 0);

        return new Matrix2(new double[][] {
                { -1, 0 },
                { 0, 1 }
        }).vmul(Matrix2.rot(base).vmul(u).add(Matrix2.rot(joint).mul(Matrix2.rot(base)).vmul(v)));
    }

    public static Pair<Values<Double>, Values<Double>> posToAngles(Vector2 pos) {
        double u = baseArmLen;
        double v = gripperArmLen;
        double a = pos.magnitude();

        double theta = Math.atan2(pos.y, pos.x);
        
        // If x < 0 and y < 0 the angle should be > pi
        if (theta < -Math.PI / 2) {
            theta = Math.PI * 2 + theta;
        }

        double phi1 = Math.acos((sq(u) - sq(v) + sq(a)) / (2 * a * u));
        double phi2 = -phi1;

        double alpha1 = theta + phi1;
        Vector2 u1Prime = Matrix2.rot(alpha1).vmul(new Vector2(u, 0));
        double dotProd1 = u1Prime.dot(pos.minus(u1Prime).neg());
        double beta1 = Math.signum(u1Prime.cross(pos.minus(u1Prime).neg()))
                * Math.acos(dotProd1 / (u1Prime.magnitude() * pos.minus(u1Prime).magnitude()));

        double alpha2 = theta + phi2;
        Vector2 u2Prime = Matrix2.rot(alpha2).vmul(new Vector2(u, 0));
        double dotProd2 = u2Prime.dot(pos.minus(u2Prime).neg());
        double beta2 = Math.signum(u2Prime.cross(pos.minus(u2Prime).neg()))
                * Math.acos(dotProd2 / (u2Prime.magnitude() * pos.minus(u2Prime).magnitude()));

        return new Pair<>(new Values<>(alpha1, -beta1), new Values<>(alpha2, -beta2));
    }
}
