package frc.robot;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import frc.fridowpi.utils.Matrix2;
import frc.fridowpi.utils.Vector2;

public class ArmCalculator {
    public static class Angles {
        public double base;
        public double joint;

        public Angles(double base, double joint) {
            this.base = base;
            this.joint = joint;
        }
    }

    public static Logger logger = LogManager.getLogger(ArmCalculator.class);

    public static class ArmState {
        public void anglesToEndPoint(Angles angles) {
        }

        public Angles endPointToAngles(Vector2 target) {
            double a2 = Constants.Arm.baseArmLength* Constants.Arm.baseArmLength;
            double b2 = Constants.Arm.gripperArmLength* Constants.Arm.gripperArmLength; 
            double c2 = target.magnitude() * target.magnitude(); 

            double alpha = Math.acos((a2 - b2 + c2) / ( 2 * Constants.Arm.baseArmLength * target.magnitude()));
            Vector2 u = Matrix2.rot(alpha).vmul(new Vector2(Constants.Arm.baseArmLength, 0.0));
            Vector2 v = target.minus(u);
            if (Math.abs(v.magnitude() - Constants.Arm.gripperArmLength) > 0.001) {
                logger.error("Calculated gripper arm did not match initial arm length");
            }

            double beta = u.angleTo(v);

            return new Angles(alpha, beta);
        }
    }

    public ArmCalculator() {

    }
}
