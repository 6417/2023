package frc.robot;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.utils.Matrix2;
import frc.fridowpi.utils.Vector2;

public class ArmCalculator implements Sendable{
    public static class MotorTorques implements Sendable {
        public double base;
        public double joint;

        public MotorTorques(double base, double joint) {
            this.base = base;
            this.joint = joint;
        }

        @Override
        public String toString() {
            return String.format("{base: %f, joint: %f}", this.base, this.joint);
        }

		@Override
		public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Base Torque: ", () -> base, null);
            builder.addDoubleProperty("Joint Torque: ", () -> joint, null);
		}
    }

    public static class AnglesSupplier implements Supplier<Angles>{
        public Supplier<Double> base;
        public Supplier<Double> joint;

        public AnglesSupplier(Supplier<Double> base, Supplier<Double> joint) {
            this.base = base;
            this.joint = joint;

        }

        public Angles get() {
            return new Angles(base.get(), joint.get());
        }
    }

    public static class Angles {
        public double base;
        public double joint;

        public Angles(double base, double joint) {
            this.base = base;
            this.joint = joint;
        }
    }

    public static Logger logger = LogManager.getLogger(ArmCalculator.class);

    public static enum Cargo {
        Cube(0.071), Cone(0.653), None(0.0);

        private final double mass; // kg

        private Cargo(double mass) {
            this.mass = mass;
        }
    }

    public static class ArmState implements Sendable {
        private AnglesSupplier angles;
        private Cargo cargo = Cargo.None;

        public ArmState(AnglesSupplier angles, Cargo cargo) {
            this.angles = angles;
            this.cargo = cargo;
        }

        public static Vector2 anglesToEndPoint(Angles angles) {
            double alpha = angles.base;
            double beta = angles.joint;

            Vector2 u = Matrix2.rot(alpha).vmul(new Vector2(Constants.Arm.baseArm.length, 0));
            if (Math.abs(u.magnitude() - Constants.Arm.baseArm.length) > 0.001) {
                logger.error("Calculated base arm did not match initial arm length");
            }
            Vector2 v = Matrix2.rot(alpha - beta).vmul(new Vector2(Constants.Arm.baseArm.length, 0));

            if (Math.abs(v.magnitude() - Constants.Arm.gripperArm.length) > 0.001) {
                logger.error("Calculated gripper arm did not match initial arm length");
            }

            return u.add(v);
        }

        public static Angles endPointToAngles(Vector2 target) {
            double a2 = Constants.Arm.baseArm.length * Constants.Arm.baseArm.length;
            double b2 = Constants.Arm.gripperArm.length * Constants.Arm.gripperArm.length;
            double c2 = target.magnitude() * target.magnitude();

            double alpha = Math.acos((a2 - b2 + c2) / (2 * Constants.Arm.baseArm.length * target.magnitude()))
                    + Math.atan2(target.y, target.x);
            Vector2 u = Matrix2.rot(alpha).vmul(new Vector2(Constants.Arm.baseArm.length, 0.0));
            Vector2 v = target.minus(u);
            if (Math.abs(v.magnitude() - Constants.Arm.gripperArm.length) > 0.001) {
                logger.error("Calculated gripper arm did not match initial arm length");
            }

            double beta = Math.signum(u.cross(v)) * u.angleTo(v);

            return new Angles(alpha, beta);
        }

        public Vector2 getPos() {
            return anglesToEndPoint(angles.get());
        }

        public Angles getAngles() {
            return angles.get();
        }

		@Override
		public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Base Angle", () -> angles.base.get(), null);
            builder.addDoubleProperty("Joint Angle", () -> angles.joint.get(), null);
            builder.addDoubleProperty("Cargo", () -> angles.joint.get(), null);
			
		}
    }

    private ArmState state;

    public ArmCalculator(ArmState initialState) {
        state = initialState;
    }

    public void updateState(ArmState newState) {
        this.state = newState;
    }

    public void updateCargo(Cargo cargo) {
        this.state.cargo = cargo;
    }

    /**
     * @param centerOfMassOffset Distance from the turning point of the base arm
     *                           to the center of mass of the object
     */
    private double baseArmTorqueOnBaseArmOf(double centerOfMassOffset, double mass) {
        return Constants.gravity * mass
                * (-Constants.Arm.baseArm.length * Math.cos(state.angles.base.get())
                        + centerOfMassOffset * Math.cos(state.angles.base.get() + state.angles.joint.get()));
    }

    /**
     * @param centerOfMassOffset Distance from the turning point of the gripper arm
     *                           to the center of mass of the object
     */
    private double baseArmTorqueOnGripperArmOf(double centerOfMassOffset, double mass) {
        return -Constants.gravity * mass *
                centerOfMassOffset * Math.cos(state.angles.base.get());
    }

    private double gripperArmTorqueOf(double centerOfMassOffset, double mass) {
        return -Constants.gravity * mass *
                centerOfMassOffset * Math.cos(state.angles.base.get() + state.angles.joint.get());
    }

    private double baseArmTorqueOfCargo() {
        return baseArmTorqueOnGripperArmOf(Constants.Arm.gripperArm.length, state.cargo.mass);
    }

    private double gripperArmTorqueOfCargo() {
        return gripperArmTorqueOf(Constants.Arm.gripperArm.length, state.cargo.mass);
    }

    private double baseArmTorqueOfBaseArm() {
        return baseArmTorqueOnBaseArmOf(Constants.Arm.baseArm.centerOfMass, Constants.Arm.baseArm.mass);
    }

    private double baseArmTorqueOfGripperArm() {
        return baseArmTorqueOnGripperArmOf(Constants.Arm.gripperArm.centerOfMass, Constants.Arm.gripperArm.mass);
    }

    private double gripperArmTorqueOfGripperArm() {
        return gripperArmTorqueOf(Constants.Arm.gripperArm.centerOfMass, Constants.Arm.gripperArm.mass);
    }

    public MotorTorques calculateTorquesForStall() {
        double baseArmTorque = Constants.Arm.baseGearRatio * baseArmTorqueOfBaseArm() + baseArmTorqueOfGripperArm() + baseArmTorqueOfCargo();
        double gripperArmTorque = Constants.Arm.jointGearRatio * gripperArmTorqueOfGripperArm() + gripperArmTorqueOfCargo();

        return new MotorTorques(baseArmTorque, gripperArmTorque);
    }

    public static double torqueToAmps(double torque) {
        return Math.abs(torque) * Constants.Arm.torqueToAmpsProportionality;
    }

	@Override
	public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Base Torque", () -> calculateTorquesForStall().base, null);
        builder.addDoubleProperty("Joint Torque", () -> calculateTorquesForStall().joint, null);
	}
}
