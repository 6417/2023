package frc.robot;

import java.awt.Container;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.command.Command;
import frc.fridowpi.utils.Matrix2;
import frc.fridowpi.utils.Vector2;

public class ArmModel implements Sendable {
    public static enum Cargo {
        None(0.0, 0.0), Cone(0.653, 0.0), Cube(0.071, 0.0); // TODO: update inertias

        public final double mass;
        public final double momentOfIntertia;

        private Cargo(double mass, double momentOfIntertia) {
            this.mass = mass;
            this.momentOfIntertia = momentOfIntertia;
        }
    }

    public static class ArmStateSupplier extends SubsystemBase {
        private class UpdateAccels extends CommandBase {
            Timer timer;
            double prevBaseVel;
            double prevGripperVel;
            
            public UpdateAccels() {
                addRequirements(ArmStateSupplier.this);
            }

            @Override
            public void initialize() {
                timer = new Timer();
                prevBaseVel = baseArmRadialVel.get();
                prevGripperVel = gripperArmRadialVel.get();
                timer.start();
            }

            @Override
            public void execute() {
                double currentBaseVel = baseArmRadialVel.get();
                double currentGripperVel = gripperArmRadialVel.get();

                baseArmRadialAccel = (currentBaseVel - prevBaseVel) / timer.get();
                gripperArmRadialAccel = (currentGripperVel - prevGripperVel) / timer.get();

                prevBaseVel = baseArmRadialVel.get();
                prevGripperVel = gripperArmRadialVel.get();
                
            }
            
            @Override
            public boolean isFinished() {
                return false;
            }
        }

        public Supplier<Double> baseArmAngle;
        public Supplier<Double> gripperArmAngle;

        public Supplier<Double> baseArmRadialVel;
        public Supplier<Double> gripperArmRadialVel;
        private double baseArmRadialAccel;
        private double gripperArmRadialAccel;

        public double getBaseArmRadialAccel() {
            return baseArmRadialAccel;
        }

        public double getGripperArmRadialAccel() {
            return gripperArmRadialAccel;
        }

        /**
         * Angle between the chassi and the center of mass of the gripper arm
         */
        public double theta() {
            double l2 = Constants.Arm.gripperArm.centerOfMass;
            double u = Constants.Arm.baseArm.length;
            double a = a();

            return baseArmAngle.get() - Math.acos((u * u + a * a - l2 * l2) / (2 * u * a));

        }

        /**
         * Distance between the base of the base arm and the center of mass of the
         * second arm
         */
        public double a() {
            double l2 = Constants.Arm.gripperArm.centerOfMass;
            double u = Constants.Arm.baseArm.length;
            double beta = gripperArmAngle.get();

            return Math.sqrt(u * u + l2 * l2 - 2 * u * l2 * Math.cos(beta));
        }

        public ArmStateSupplier(Supplier<Double> baseArmAngle,
                Supplier<Double> gripperArmAngle,
                Supplier<Double> baseArmRadialVel,
                Supplier<Double> gripperArmRadialVel) {
            this.baseArmAngle = baseArmAngle;
            this.baseArmRadialVel = baseArmRadialVel;
            this.gripperArmAngle = () -> gripperArmAngle.get() + Constants.Arm.jointCenterOfMassAngleOffset;
            this.gripperArmRadialVel = gripperArmRadialVel;
            
            this.setDefaultCommand(new UpdateAccels());
        }
    }

    public ArmStateSupplier state;
    private Cargo cargo;

    public double getGripperArmMomentOfInertia() {
        return Math.pow(Constants.Arm.gripperArm.length, 2) * cargo.mass + Constants.Arm.gripperArm.momentOfInertia;
    }

    public ArmModel(ArmStateSupplier state, Cargo initialCargo) {
        this.state = state;
        cargo = initialCargo;
    }

    public double torqueOfBaseArmToGripperArm() {
        return Constants.Arm.baseArm.momentOfInertia * state.getGripperArmRadialAccel();
    }

    private double getGripperArmMass() {
        return Constants.Arm.gripperArm.mass + cargo.mass;
    }

    private double getGriArmCenterOfMass() {
        return (Constants.Arm.gripperArm.mass * Constants.Arm.gripperArm.centerOfMass
                + cargo.mass * Constants.Arm.gripperArm.length) / (Constants.Arm.gripperArm.mass + cargo.mass);
    }

    private double torqueOfGravityGripperArm() {
        // return Math.cos(state.theta()) * Constants.gravity * (getGripperArmMass() +
        // getGripperArmMass())
        // * getGriArmCenterOfMass();
        //
        double alpha = state.baseArmAngle.get();
        double beta = state.gripperArmAngle.get();
        double phi = beta - alpha;
        
        return Constants.gravity * getGripperArmMass() * getGriArmCenterOfMass() * Math.cos(phi);

        // Matrix2 betaRot = Matrix2.rot(beta - Math.PI / 2.0);
        // Matrix2 alphaRot = Matrix2.rot(alpha);

        // double scalar = new Vector2(0, 1.0).dot(betaRot.vmul((alphaRot.vmul(new Vector2(0, -1.0)))));
        
        // return (Math.cos(alpha + beta)) * Constants.gravity
        //         * getGripperArmMass() * getGriArmCenterOfMass();
    }

    private double torqueOfGravityBaseArm() {
        return Math.cos(state.baseArmAngle.get()) * Constants.gravity * Constants.Arm.baseArm.mass
                * Constants.Arm.baseArm.centerOfMass;
    }

    public Pair<Double, Double> torqueToHoldState() {
        double mgGripper = torqueOfGravityGripperArm();
        // double mgBase = torqueOfGravityBaseArm();
        double mgBase = 0;

        return new Pair<>(mgBase, mgGripper + torqueOfBaseArmToGripperArm());
    }

    public static double torqueToAmps(double torque) {
        return Math.abs(torque) * Constants.Arm.torqueToAmpsProportionality / 2.0;
    }

    public static double ampsToTorque(double amps) {
        return amps / Constants.Arm.torqueToAmpsProportionality;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Radial acceleration base arm", state::getBaseArmRadialAccel, null);
        builder.addDoubleProperty("Radial acceleration gripper arm", state::getGripperArmRadialAccel, null);
        builder.addStringProperty("Cargo", () -> this.cargo.toString(), null);
        builder.addDoubleProperty("Base Arm Torque to Hold State [Nm]", () -> torqueToHoldState().getFirst(), null);
        builder.addDoubleProperty("Gripper Arm Torque to Hold State [Nm]", () -> torqueToHoldState().getSecond(), null);

        builder.addDoubleProperty("Base Arm Torque to Hold State [A]",
                () -> torqueToAmps(torqueToHoldState().getFirst()) * Constants.Arm.baseGearRatio / 2, null);
        builder.addDoubleProperty("Gripper Arm Torque to Hold State [A]",
                () -> torqueToAmps(torqueToHoldState().getSecond()) * Constants.Arm.jointGearRatio / 2, null);

        builder.addDoubleProperty("Base Arm Torque of Gravity [Nm]", () -> torqueOfGravityBaseArm(), null);
        builder.addDoubleProperty("Gripper Arm Torque of Gravity [Nm]", () -> torqueOfGravityGripperArm(), null);

        builder.addDoubleProperty("Gripper Arm Torque of Base Arm [Nm]", () -> torqueOfBaseArmToGripperArm(), null);
        builder.addDoubleProperty("a [m]", () -> state.a(), null);
        builder.addDoubleProperty("theta [DEG]", () -> state.theta() * 180 / Math.PI, null);
        builder.addDoubleProperty("Max Gravity Torque [Nm]",
                () -> Constants.gravity * getGripperArmMass() * getGriArmCenterOfMass(), null);

        builder.addDoubleProperty("Gravity multiplier [1]", () -> {
            double alpha = state.baseArmAngle.get();
            double beta = state.gripperArmAngle.get();
            return Math.cos(alpha + beta);
        }, null);
    }

    public void updateCargo(Cargo cargo) {
        this.cargo = cargo;
    }
    
    public Cargo getCargo() {
        return cargo;
    }
}
