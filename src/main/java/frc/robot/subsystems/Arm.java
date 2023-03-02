package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.robot.ArmModel;
import frc.robot.Constants;
import frc.robot.commands.arm.BaseGotoPositionShuffleBoard;
import frc.robot.commands.arm.BaseManualControl;
import frc.robot.commands.arm.JointManualControl;
import frc.robot.commands.arm.ResetBaseEncodersOnHall;
import frc.robot.commands.arm.ResetBaseEncodersOnLimitSwitch;
import frc.robot.commands.arm.ToggleCone;
import frc.robot.subsystems.base.ArmBase;
import jdk.jfr.Percentage;

public class Arm extends ArmBase {
    private static ArmBase instance = null;

    private class Motors {
        public FridoFalcon500 base;
        private FridoFalcon500 baseFollower;
        public FridoFalcon500 joint;
        private FridoFalcon500 jointFollower;

        public Motors() {
            base = new FridoFalcon500(Constants.Arm.Ids.baseMotor);
            baseFollower = new FridoFalcon500(Constants.Arm.Ids.baseFollowerMotor);

            joint = new FridoFalcon500(Constants.Arm.Ids.jointMotor);
            jointFollower = new FridoFalcon500(Constants.Arm.Ids.jointFollowerMotor);

            base.setIdleMode(IdleMode.kBrake);
            baseFollower.setIdleMode(IdleMode.kBrake);
            joint.setIdleMode(IdleMode.kBrake);
            jointFollower.setIdleMode(IdleMode.kBrake);

            baseFollower.follow(base, Constants.Arm.baseFollowType);
            jointFollower.follow(joint, Constants.Arm.jointFollowType);

            base.configEncoder(FridoFeedBackDevice.kBuildin, 2048);
            joint.configEncoder(FridoFeedBackDevice.kBuildin, 2048);

            base.setInverted(true);

            base.enableForwardLimitSwitch(Constants.Arm.limitSwitchPolarity, true);
            baseFollower.enableForwardLimitSwitch(Constants.Arm.limitSwitchPolarity, true);

            base.enableReverseLimitSwitch(Constants.Arm.limitSwitchPolarity, true);
            baseFollower.enableReverseLimitSwitch(Constants.Arm.limitSwitchPolarity, true);

            base.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                    Constants.Arm.baseStatorCurrentLimit, Constants.Arm.baseStatorCurrentLimit, 1));
            baseFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                    Constants.Arm.baseStatorCurrentLimit, Constants.Arm.baseStatorCurrentLimit, 1));

            joint.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                    Constants.Arm.baseStatorCurrentLimit, Constants.Arm.baseStatorCurrentLimit, 1));
            jointFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
                    Constants.Arm.jointStatorCurrentLimit, Constants.Arm.jointStatorCurrentLimit, 1));
            
            joint.configNeutralDeadband(0.0);
            jointFollower.configNeutralDeadband(0.0);

            base.config_kP(0, Constants.Arm.basePid.p);
            base.config_kI(0, Constants.Arm.basePid.i);
            base.config_kD(0, Constants.Arm.basePid.d);
            base.config_IntegralZone(0, Constants.Arm.basePid.iZone);
            base.setIntegralAccumulator(Constants.Arm.basePid.maxIntegralAccum, 0, 20);
            base.configAllowableClosedloopError(0, Constants.Arm.basePid.allowableError);

            base.configMotionCruiseVelocity(Constants.Arm.baseMotionMagic.cruiseVel);
            base.configMotionAcceleration(Constants.Arm.baseMotionMagic.accel);
            base.configMotionSCurveStrength(Constants.Arm.baseMotionMagic.curveStrength);
        }
    }

    private Motors motors;
    private ArmModel model;
    private boolean isBaseZeroed = false;
    private boolean isJointZeroed = false;
    private boolean holdJoint = true;
    private PIDController jointPosController;
    private DigitalInput baseHallRight;
    private DigitalInput baseHallLeft;

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

    double kP = 0.0; // 0.2
    double kI = 0.0; // 0.000025 * factor;
    double kD = 0.0; // 0.3

    @Override
    public void init() {
        motors = new Motors();
        model = new ArmModel(
                new ArmModel.ArmStateSupplier(this::baseAngle, this::jointAngle, () -> 0.0, () -> 0.0),
                Constants.Arm.initialCargo);
        jointPosController = new PIDController(kP, kI, kD, 10);

        baseHallRight = new DigitalInput(Constants.Arm.dioIdHallBaseRight);
        baseHallLeft = new DigitalInput(Constants.Arm.dioIdHallBaseLeft);

        Shuffleboard.getTab("Arm").add(this);
        Shuffleboard.getTab("Arm").add("PID", jointPosController);
        Shuffleboard.getTab("Arm").add("Model", model);
        CommandScheduler.getInstance().schedule(new ResetBaseEncodersOnLimitSwitch(), new ResetBaseEncodersOnHall());

        CommandBase manualControl = new ParallelCommandGroup(new BaseManualControl(), new JointManualControl());
        manualControl.addRequirements(this);
        setDefaultCommand(manualControl);
    }

    @Override
    public double baseAngle() {
        return motors.base.getEncoderTicks() / Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.baseGearRatio * Math.PI * 2.0;
    }


    public void setJointTargetPos(double pos) {
        jointPosController.reset();
        jointPosController.calculate(motors.joint.getEncoderTicks(), pos);
    }

    public double jointAngle() {
        return motors.joint.getEncoderTicks() /
                Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.jointGearRatio * Math.PI * 2.0;
    }

    @Override
    public ArmModel getModel() {
        return model;
    }

    @Override
    public double getBaseEncoderVelocity() {
        return motors.base.getEncoderVelocity();
    }

    @Override
    public void setPercentBase(double percent) {
        motors.base.set(percent);
    }

    @Override
    public void setPercentJoint(double percent) {
        motors.joint.set(percent);
    }

    @Override
    public void setEncoderTicksBase(double ticks) {
        isBaseZeroed = true;
        motors.base.setEncoderPosition(ticks);
    }

    @Override
    public void setEncoderTicksJoint(double ticks) {
        isJointZeroed = true;
        motors.joint.setEncoderPosition(ticks);
    }

    @Override
    public boolean getBaseLimitSwitchFwd() {
        boolean value = motors.base.isForwardLimitSwitchActive();
        if (Constants.Arm.limitSwitchPolarity == LimitSwitchPolarity.kNormallyClosed) {
            return !value;
        } else {
            return value;
        }
    }

    @Override
    public boolean getBaseLimitSwitchRev() {
        boolean value = motors.base.isReverseLimitSwitchActive();
        if (Constants.Arm.limitSwitchPolarity == LimitSwitchPolarity.kNormallyClosed) {
            return !value;
        } else {
            return value;
        }
    }

    public static double baseAngleToTicks(double angle) {
        final double minTicks = Math.min(Constants.Arm.baseEncoderPosFwdLimitSwitch,
                Constants.Arm.baseEncoderPosRevLimitSwitch);
        final double maxTicks = Math.max(Constants.Arm.baseEncoderPosFwdLimitSwitch,
                Constants.Arm.baseEncoderPosRevLimitSwitch);

        return MathUtil.clamp(angle / (2 * Math.PI) * 2048 / Constants.Arm.baseGearRatio, minTicks, maxTicks);
    }

    @Override
    public void baseGotoAngle(double angle) {
        if (isBaseZeroed) {
            motors.base.set(ControlMode.MotionMagic, baseAngleToTicks(angle));
        } else {
            DriverStation.reportError("Can't go to an angle if base arm is not zeroed", false);
        }
    }

    @Override
    public boolean baseIsAtTarget() {
        return true;
    }

    @Override
    public void setBasePercent(double percent) {
        motors.base.set(percent);
    }

    @Override
    public void setJointPercent(double percent) {
        motors.joint.set(percent);
    }

    @Override
    public void stop() {
        motors.base.stopMotor();
        motors.joint.stopMotor();
    }

    @Override
    public List<Binding> getMappings() {
        return List
                .of(new Binding(Constants.Joysticks.armJoystick, Logitech.b, Button::toggleOnTrue, new ToggleCone()),
                        new Binding(Constants.Joysticks.armJoystick, Logitech.start, Button::onTrue,
                                new InstantCommand(() -> {
                                    isBaseZeroed = !(isBaseZeroed && isJointZeroed);
                                    isJointZeroed = !(isBaseZeroed && isJointZeroed);
                                })));
    }

    double currentVelOut;
    double kF = 0.001;
    double iZone = 1000;

    @Override
    public void periodic() {
        if (isJointZeroed && isBaseZeroed) {
            double torque = model.torqueToHoldState().getSecond();

            if (holdJoint) {
                
                motors.joint.set(torque * kF);
            } else {
                if (jointPosController.getPositionError() > 1000) {
                    jointPosController.setIntegratorRange(0, 0);
                } else {
                    jointPosController.setIntegratorRange(-1, 1);
                }

                double posOut = MathUtil.clamp(jointPosController.calculate(motors.joint.getEncoderTicks()), -0.3, 0.3);
                currentVelOut = posOut;
                motors.joint.set(torque * kF + posOut);
            }
        }
    }

    @Override
    public void enableHoldJoint() {
        if (!holdJoint) {
            holdJoint = true;
            jointPosController.reset();
        }
    }

    @Override
    public void reset() {
        jointPosController.reset();
    }

    @Override
    public void disableHoldJoint() {
        if (holdJoint) {
            holdJoint = false;
            motors.joint.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.Arm.jointMaxAmps,
                    Constants.Arm.jointMaxAmps, 1), 1);

            jointPosController.reset();
        }
    }

    @Override
    public boolean isZeroed() {
        return isBaseZeroed && isJointZeroed;
    }

    @Override
    public boolean isBaseLeftHallActive() {
        return baseHallRight.get();
    }

    @Override
    public boolean isBaseRightHallActive() {
        return baseHallLeft.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Torque to hold state base", () -> model.torqueToHoldState().getFirst(), null);
        builder.addDoubleProperty("Torque to hold state joint", () -> model.torqueToHoldState().getSecond(), null);
        builder.addBooleanProperty("Torque to hold state base direction",
                () -> model.torqueToHoldState().getFirst() > 0, null);
        builder.addBooleanProperty("Torque to hold state joint direction",
                () -> model.torqueToHoldState().getSecond() > 0, null);
        builder.addDoubleProperty("Base Angle [DEG]", () -> Math.toDegrees(this.baseAngle()), null);
        builder.addDoubleProperty("Joint Angle [DEG]", () -> Math.toDegrees(this.jointAngle()), null);
        builder.addDoubleProperty("Joint ticks", motors.joint::getEncoderTicks, null);
        builder.addDoubleProperty("Base ticks", motors.base::getEncoderTicks, null);
        builder.addDoubleProperty("Motor Temp base right", motors.base::getTemperature, null);
        builder.addDoubleProperty("Motor Temp base left", motors.baseFollower::getTemperature, null);
        builder.addDoubleProperty("Motor Temp joint right", motors.joint::getTemperature, null);
        builder.addDoubleProperty("Motor Temp joint left", motors.jointFollower::getTemperature, null);
        builder.addDoubleProperty("Joint Current Stator", motors.joint::getTemperature, null);
        builder.addDoubleProperty("Joint encoder velocity", motors.joint::getEncoderVelocity, null);
        builder.addBooleanProperty("Base hall right", this::isBaseRightHallActive, null);
        builder.addBooleanProperty("Base hall left", this::isBaseLeftHallActive, null);
        builder.addDoubleProperty("Base CurrentStator", motors.base::getStatorCurrent, null);
        builder.addDoubleProperty("kF", () -> kF, (newF) -> kF = newF );
        builder.addDoubleProperty("out hold", () -> kF * model.torqueToHoldState().getSecond(), null);
        builder.addDoubleProperty("VelOut", () -> currentVelOut, null);
        builder.addDoubleProperty("joint target pos", () -> jointPosController.getSetpoint(), (target) -> this.setJointTargetPos(target));
        builder.addBooleanProperty("Base is zeroed", () -> isBaseZeroed, null);
        builder.addDoubleProperty("base shuffle board target",
                () -> Math.toDegrees(BaseGotoPositionShuffleBoard.getInstance().target),
                (newTarget) -> BaseGotoPositionShuffleBoard.getInstance().target = Math.toRadians(newTarget));
    }
}