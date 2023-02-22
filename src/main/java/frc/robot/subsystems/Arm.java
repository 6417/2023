package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.ArmCalculator;
import frc.robot.Constants;
import frc.robot.subsystems.base.ArmBase;
import jdk.jfr.Percentage;

public class Arm extends ArmBase {
    private static ArmBase instance = null;

    private class Motors {
        public FridoFalcon500 base;
        // private FridoFalcon500 baseFollower;
        // public FridoFalcon500 joint;
        // private FridoFalcon500 jointFollower;

        public Motors() {
            base = new FridoFalcon500(Constants.Arm.Ids.baseMotor);
            // baseFollower = new FridoFalcon500(Constants.Arm.Ids.baseFollowerMotor);
            //
            // joint = new FridoFalcon500(Constants.Arm.Ids.jointMotor);
            // jointFollower = new FridoFalcon500(Constants.Arm.Ids.jointFollowerMotor);

            base.setIdleMode(IdleMode.kBrake);
            // baseFollower.setIdleMode(IdleMode.kBrake);
            // joint.setIdleMode(IdleMode.kBrake);
            // jointFollower.setIdleMode(IdleMode.kBrake);

            // baseFollower.follow(base, Constants.Arm.baseFollowType);
            // jointFollower.follow(joint, Constants.Arm.jointFollowType);

            base.configEncoder(FridoFeedBackDevice.kBuildin, 2048);
            // joint.configEncoder(FridoFeedBackDevice.kBuildin, 2048);

            base.enableForwardLimitSwitch(Constants.Arm.limitSwitchPolarity, true);
            // joint.enableForwardLimitSwitch(Constants.Arm.limitSwitchPolarity, true);
        }

        public void setCurrentLimitBase(double amps) {
            base.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, amps, amps, 0.001), 0);
            // baseFollower.configStatorCurrentLimit(new
            // StatorCurrentLimitConfiguration(true, amps, amps, 0.001),0);
        }

        public void setCurrentLimitJoint(double amps) {
            // joint.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
            // amps, amps, 0.001),0);
            // jointFollower.configStatorCurrentLimit(new
            // StatorCurrentLimitConfiguration(true, amps, amps, 0.001),0);
        }

    }

    private Motors motors;
    private ArmCalculator calculator;

    private double currentLimit = Double.POSITIVE_INFINITY;

    public static ArmBase getInstance() {
        if (instance == null) {

            if (Constants.Arm.enabled) {
                instance = new Arm();
            } else {
                instance = new ArmBase();
            }
        }
        return instance;
    }

    @Override
    public void init() {
        motors = new Motors();
        calculator = new ArmCalculator(
                new ArmCalculator.ArmState(new ArmCalculator.AnglesSupplier(this::baseAngle, this::jointAngle),
                        Constants.Arm.initialCargo));

        addChild("Arm State", calculator);
        Shuffleboard.getTab("Arm").add(this);
    }

    @Override
    public double baseAngle() {
        return motors.base.getEncoderTicks() / Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.baseGearRatio * Math.PI * 2.0;
    }

    // public double jointAngle() {
    // return motors.joint.getEncoderTicks() /
    // Constants.Arm.encoderTicksPerMotorRevolution
    // * Constants.Arm.jointGearRatio * Math.PI * 2.0;
    // }

    public ArmCalculator getCalculator() {
        return calculator;
    }

    public void setBaseAmpsLimit(double amps) {
        currentLimit = amps;
        motors.setCurrentLimitBase(amps);
    }

    @Override
    public void setPercentBase(double percent) {
        motors.base.set(percent);
    }

    @Override
    public void resetEncodersBase() {
        motors.base.setEncoderPosition(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Amps", () -> currentLimit, null);
        builder.addDoubleProperty("Torque", () -> calculator.calculateTorquesForStall().base, null);
        builder.addBooleanProperty("Direction", () -> calculator.calculateTorquesForStall().base > 0, null);
        builder.addDoubleProperty("Base Angle", this::baseAngle, null);
        builder.addDoubleProperty("Encoder Ticks", motors.base::getEncoderTicks, null);
        builder.addDoubleProperty("Motor Temp", motors.base::getTemperature, null);
    }
}
