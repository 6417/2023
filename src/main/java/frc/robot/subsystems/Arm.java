package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay.InvalidValueException;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.utils.LatchedBooleanRising;
import frc.fridowpi.utils.LatchedBooleanFalling;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmKinematics;
import frc.robot.ArmModel;
import frc.robot.ArmPathGenerator;
import frc.robot.ArmPosJoystick;
import frc.robot.Constants;
import frc.robot.ArmPathGenerator.RobotOrientation;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.commands.InstantCommandRunWhenDisabled;
import frc.robot.commands.arm.GotoPos;
import frc.robot.commands.arm.IndividualArmManualControl;
import frc.robot.commands.arm.ManualPosControl;
import frc.robot.commands.arm.RawManualControl;
import frc.robot.commands.arm.ResetBaseEncodersOnHall;
import frc.robot.commands.arm.ResetBaseEncodersOnLimitSwitch;
import frc.robot.commands.arm.ResetJointEncodersOnDistanceSensor;
import frc.robot.commands.arm.ZeroBase;
import frc.robot.commands.arm.ZeroJoint;
import frc.robot.subsystems.base.ArmBase;

public class Arm extends ArmBase {
    private static ArmBase instance = null;

    public static enum ManualControlMode {
        RAW(new RawManualControl()),
        INDIVIDUAL_ARMS(new IndividualArmManualControl()),
        POS(new ManualPosControl());

        public final Command command;

        private ManualControlMode(CommandBase cmd) {
            this.command = cmd;
        }
    }

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

            joint.configNeutralDeadband(0.0001);
            jointFollower.configNeutralDeadband(0.0001);

            base.config_kP(0, Constants.Arm.basePid.p);
            base.config_kI(0, Constants.Arm.basePid.i);
            base.config_kD(0, Constants.Arm.basePid.d);
            base.config_IntegralZone(0, Constants.Arm.basePid.iZone);
            base.setIntegralAccumulator(Constants.Arm.basePid.maxIntegralAccum, 0, 20);
            base.configAllowableClosedloopError(0, Constants.Arm.basePid.allowableError);

            base.configMotionCruiseVelocity(Constants.Arm.baseMotionMagic.cruiseVel);
            base.configMotionAcceleration(Constants.Arm.baseMotionMagic.accel);
            base.configMotionSCurveStrength(Constants.Arm.baseMotionMagic.curveStrength);

            joint.configForwardSoftLimitThreshold(21000);
            jointFollower.configForwardSoftLimitThreshold(21000);

            joint.configReverseSoftLimitThreshold(-28100);
            jointFollower.configReverseSoftLimitThreshold(-28100);

            joint.config_kP(0, Arm.this.kP);
            joint.config_kI(0, Arm.this.kI);
            joint.config_kD(0, Arm.this.kD);
            joint.config_kF(0, 0);

            joint.configMotionAcceleration(9000);
            joint.configMotionCruiseVelocity(3000); // TODO: change back to 3000
        }
    }

    private Motors motors;
    private ArmModel model;
    private boolean baseZeroed = false;
    private boolean jointZeroed = false;
    private boolean holdJoint = true;
    private PIDController jointPosController;
    private ManualControlMode currentManualControlMode;
    private ArmPathGenerator pathGenerator;
    private ArmPathGenerator.RobotOrientation orientation;
    private ArmPathGenerator.RobotPos pos;

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

    double kP = 0.045; // 0.2
    double kI = 0.00001; // 25; // 0.000025 * factor;
    double kD = 0.001; // 0.3

    LatchedBooleanRising gettingZeroed;
    LatchedBooleanFalling gettingUnzeroed;

    AnalogInput distanceSensorGripperArm;

    @Override
    public void init() {
        distanceSensorGripperArm = new AnalogInput(Constants.Arm.gripperArmDistanceSensorAnalogInputPin);
        pos = RobotPos.FIELD;
        orientation = RobotOrientation.FORWARD;
        pathGenerator = new ArmPathGenerator();
        gettingZeroed = new LatchedBooleanRising(baseZeroed && jointZeroed);
        gettingUnzeroed = new LatchedBooleanFalling(baseZeroed && jointZeroed);
        motors = new Motors();
        model = new ArmModel(
                new ArmModel.ArmStateSupplier(this::baseAngle, this::jointAngle, () -> 0.0, () -> 0.0),
                Constants.Arm.initialCargo);
        jointPosController = new PIDController(kP, kI, kD, 10);

        Shuffleboard.getTab("Arm").add(this);
        Shuffleboard.getTab("Arm").add("PID", jointPosController);
        Shuffleboard.getTab("Arm").add("ZeroBase", new ZeroBase());
        Shuffleboard.getTab("Arm").add("ZeroJoint", new ZeroJoint());
        Shuffleboard.getTab("Arm").add("Model", model);
        CommandScheduler.getInstance().schedule(new ResetBaseEncodersOnLimitSwitch(),
                new ResetJointEncodersOnDistanceSensor());

        Thread resetOnHall = new Thread(() -> {
            DigitalInput baseHallRight = new DigitalInput(Constants.Arm.dioIdHallBaseRight);
            DigitalInput baseHallLeft = new DigitalInput(Constants.Arm.dioIdHallBaseLeft);
            Command cmd = new ResetBaseEncodersOnHall(baseHallRight::get, baseHallLeft::get);

            cmd.initialize();
            while (!cmd.isFinished()) {
                cmd.execute();
                try {
                    Thread.sleep(1);
                } catch (Exception e) {

                }
            }
            cmd.end(false);
            baseHallRight.close();
            baseHallLeft.close();
        });

        resetOnHall.start();

        currentManualControlMode = ManualControlMode.RAW;
        setDefaultCommand(currentManualControlMode.command);
    }

    @Override
    public void setManualControlMode(ManualControlMode mode) {
        if (!isZeroed() && mode != ManualControlMode.RAW) {
            mode = ManualControlMode.RAW;
            DriverStation.reportError("Can't use other mode then RAW if arm is not zeroed, mode will NOT be switched.",
                    false);
        }

        CommandScheduler.getInstance().removeDefaultCommand(this);
        CommandScheduler.getInstance().cancel(currentManualControlMode.command);
        currentManualControlMode = mode;
        setDefaultCommand(currentManualControlMode.command);
    }

    @Override
    public double baseAngle() {
        return motors.base.getEncoderTicks() / Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.baseGearRatio * Math.PI * 2.0;
    }

    private double targetPosJoint = 0;
    private double targetPosBase = (Constants.Arm.baseEncoderPosFwdLimitSwitch
            + Constants.Arm.baseEncoderPosRevLimitSwitch) / 2;

    private void setJointTargetPos(double pos) {
        motors.joint.set(ControlMode.MotionMagic, pos);
        targetPosJoint = pos;
    }

    private void setBaseTargetPos(double pos) {
        motors.base.set(ControlMode.MotionMagic, pos);
        targetPosBase = pos;
    }

    @Override
    public void hold() {
        baseGotoAngle(baseAngle());
        jointGotoAngle(jointAngle());
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
        baseZeroed = true;
        motors.base.setEncoderPosition(ticks);
    }

    @Override
    public void setEncoderTicksJoint(double ticks) {
        jointZeroed = true;
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
        return angle / (2 * Math.PI) * 2048 / Constants.Arm.baseGearRatio;
    }

    public static double jointAngleToTicks(double angle) {
        return angle / (2 * Math.PI) * 2048 / Constants.Arm.jointGearRatio;
    }

    public static double baseTicksToAngle(double ticks) {
        return ticks * (2 * Math.PI) / 2048 * Constants.Arm.baseGearRatio;
    }

    public static double jointTicksToAngle(double ticks) {
        return ticks * (2 * Math.PI) / 2048 * Constants.Arm.jointGearRatio;
    }

    @Override
    public void baseGotoAngle(double angle, boolean fine) {
        if (Double.isFinite(angle) && !Double.isNaN(angle)) {
            if (baseZeroed && jointZeroed) {
                setBaseTargetPos(baseAngleToTicks(angle));
            } else {
                DriverStation.reportError("Can't go to an angle if base arm is not zeroed", false);
            }
        } else {
            DriverStation.reportError("[Arm::baseGotoAngle] received angle: " + angle, false);
        }
    }

    @Override
    public void baseGotoAngleNoZeroedCheckOfJoint(double angle) {
        if (Double.isFinite(angle) && !Double.isNaN(angle)) {
            if (baseZeroed) {
                setBaseTargetPos(baseAngleToTicks(angle));
            } else {
                DriverStation.reportError("Can't go to an angle if base arm is not zeroed", false);
            }
        } else {
            DriverStation.reportError("[Arm::baseGotoAngle] received angle: " + angle, false);
        }
    }

    @Override
    public void jointGotoAngle(double angle) {
        if (Double.isFinite(angle) && !Double.isNaN(angle)) {
            if (baseZeroed && jointZeroed) {
                setJointTargetPos(jointAngleToTicks(angle));
            } else {
                DriverStation.reportError("Can't go to an angle if base arm is not zeroed", false);
            }
        } else {
            DriverStation.reportError("[Arm::jointGotoAngle] received angle: " + angle, false);
        }
    }

    @Override
    public double getBaseTargetAngle() {
        return baseTicksToAngle(targetPosBase);
    }

    @Override
    public double getJointTargetAngle() {
        return jointTicksToAngle(targetPosJoint);
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
    public boolean isPosValid(Vector2 pos) {
        return pathGenerator.isValidInField(pos);
    }

    @Override
    public ArmPathGenerator.RobotOrientation getRobotOrientation() {
        return orientation;
    }

    @Override
    public RobotPos getRobotPos() {
        return pos;
    }

    @Override
    public void setRobotOrientation(RobotOrientation orientation) {
        this.orientation = orientation;
    }

    @Override
    public void setRobotPos(RobotPos pos) {
        this.pos = pos;
    }

    @Override
    public List<Binding> getMappings() {
        List<Binding> posBindings = Arrays.stream(ArmPosJoystick.Ids.values()).map((id) -> {
            return new Binding(Constants.Joysticks.armJoystick, id, Button::onTrue,
                    new GotoPos(id.target, id.pos, id.orientation, id.toString()));
        }).collect(Collectors.toList());
        List<Binding> otherBindings = List
                .of(new Binding(Constants.Joysticks.armJoystick, Logitech.lb, Button::onFalse, new InstantCommand(
                        () -> Arm.getInstance().setEncoderTicksJoint(-167.0 / 360.0 /
                                Constants.Arm.jointGearRatio * 2048))),
                        new Binding(Constants.Joysticks.armJoystick, Logitech.start, Button::onTrue,
                                new InstantCommandRunWhenDisabled(() -> {
                                    int modeIdx = List.of(ManualControlMode.values()).indexOf(currentManualControlMode);

                                    stop();
                                    setManualControlMode(
                                            ManualControlMode.values()[(modeIdx + 1)
                                                    % ManualControlMode.values().length]);
                                })),
                        new Binding(Constants.Joysticks.armJoystick, Logitech.rb, Button::onTrue,
                                new InstantCommandRunWhenDisabled(() -> {
                                    boolean zeroed = !(isJointZeroed() || isBaseZeroed());
                                    baseZeroed = zeroed;
                                    jointZeroed = zeroed;
                                })),

                        new Binding(Constants.Joysticks.armJoystick, Logitech.back, Button::onTrue,
                                new InstantCommandRunWhenDisabled(() -> {
                                    int modeIdx = List.of(ManualControlMode.values()).indexOf(currentManualControlMode);
                                    stop();
                                    setManualControlMode(
                                            ManualControlMode.values()[(modeIdx + ManualControlMode.values().length - 1)
                                                    % ManualControlMode.values().length]);
                                })));

        return Stream.concat(posBindings.stream(), otherBindings.stream()).toList();
    }

    double currentVelOut;
    final double kF = 0.002;
    double iZone = 1000;

    int lastValidQuadrant = -1;

    @Override
    public int getLastValidQuadrant() {
        return lastValidQuadrant;
    }

    @Override
    public void enableBreakModeJoint() {
        motors.joint.setIdleMode(IdleMode.kBrake);
        motors.jointFollower.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void disableBreakModeJoint() {
        motors.joint.setIdleMode(IdleMode.kCoast);
        motors.jointFollower.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        if (gettingUnzeroed.updateAndGet(isZeroed())) {
            setManualControlMode(ManualControlMode.RAW);
        }

        if (isZeroed()) {
            try {
                lastValidQuadrant = pathGenerator.getQuadrantIndexOfPoint(getPos(), pos, orientation);
            } catch (InvalidValueException ignored) {
                // Invalid position not updating
            }
        }

        if (Math.abs(jointAngle()) < Math.abs(jointTicksToAngle(motors.joint.getEncoderVelocity()) * 7.5)
                + Math.toRadians(5)) {
            if (Math.signum(motors.joint.getEncoderVelocity()) != Math.signum(jointAngle())
                    && motors.joint.getEncoderVelocity() != 0) {
                GripperSubsystem.getInstance().closeGripper();
            }
        }

        if (jointZeroed && baseZeroed) {
            double torque = model.torqueToHoldState().getSecond();

            motors.joint.config_kF(0, MathUtil.clamp(torque * kF, -0.1, 0.1), 0);
        }
    }

    @Override
    public boolean isZeroed() {
        return baseZeroed && jointZeroed;
    }

    @Override
    public boolean isBaseAtTargetDriveThrough() {
        return Math.abs(motors.base.getEncoderTicks() - targetPosBase) < Constants.Arm.baseAllowableErrorDriveThrough;
    }

    @Override
    public boolean isJointAtTargetDriveThrough() {
        return Math
                .abs(motors.joint.getEncoderTicks() - targetPosJoint) < Constants.Arm.jointAllowableErrorDriveThrough;
    }

    @Override
    public boolean isBaseAtTarget() {
        return Math.abs(motors.base.getEncoderTicks() - targetPosBase) < Constants.Arm.baseAllowableError;
    }

    @Override
    public boolean isJointAtTarget() {
        return Math.abs(motors.joint.getEncoderTicks() - targetPosJoint) < Constants.Arm.jointAllowableError;
    }

    @Override
    public boolean isArmAtTargetDriveThrough() {
        return isBaseAtTargetDriveThrough() && isJointAtTargetDriveThrough();
    }

    @Override
    public boolean isJointZeroed() {
        return jointZeroed;
    }

    @Override
    public boolean isBaseZeroed() {
        return baseZeroed;
    }

    @Override
    public boolean isArmAtTarget() {
        return isBaseAtTarget() && isJointAtTarget();
    }

    @Override
    public Vector2 getPos() {
        return ArmKinematics.anglesToPos(baseAngle(), jointAngle());
    }

    @Override
    public double getGripperArmDistanceSensor() {
        return distanceSensorGripperArm.getAverageVoltage();
    }

    @Override
    public double getJointEncoderVelocity() {
        return motors.joint.getEncoderVelocity();
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
        builder.addBooleanProperty("holdJoint", () -> holdJoint, null);
        builder.addDoubleProperty("Base ticks", motors.base::getEncoderTicks, null);
        // builder.addDoubleProperty("Motor Temp base right",
        // motors.base::getTemperature, null);
        // builder.addDoubleProperty("Motor Temp base left",
        // motors.baseFollower::getTemperature, null);
        // builder.addDoubleProperty("Motor Temp joint right",
        // motors.joint::getTemperature, null);
        // builder.addDoubleProperty("Motor Temp joint left",
        // motors.jointFollower::getTemperature, null);
        // builder.addDoubleProperty("Joint Current Stator",
        // motors.joint::getTemperature, null);
        builder.addDoubleProperty("Joint encoder velocity", motors.joint::getEncoderVelocity, null);
        builder.addDoubleProperty("Base CurrentStator", motors.base::getStatorCurrent, null);
        builder.addDoubleProperty("out hold", () -> kF * model.torqueToHoldState().getSecond(), null);
        builder.addDoubleProperty("VelOut", () -> currentVelOut, null);
        builder.addBooleanProperty("Base is zeroed", () -> baseZeroed, null);
        builder.addBooleanProperty("Joint is zeroed", () -> jointZeroed, null);
        builder.addDoubleProperty("Arm pos x", () -> ArmKinematics.anglesToPos(baseAngle(), jointAngle()).x, null);
        builder.addDoubleProperty("Arm pos y", () -> ArmKinematics.anglesToPos(baseAngle(), jointAngle()).y, null);
        builder.addStringProperty("Robot Pos", () -> pos.toString(), null);
        builder.addStringProperty("Robot Orientation", () -> orientation.toString(), null);
        builder.addStringProperty("Manual Control Mode", () -> currentManualControlMode.toString(), null);
        builder.addDoubleProperty("Pos Quadrant idx", () -> {
            try {
                return pathGenerator.getQuadrantIndexOfPoint(getPos(), RobotPos.GRID, RobotOrientation.FORWARD);
            } catch (InvalidValueException e) {
                return -1;
            }
        }, null);
        builder.addDoubleProperty("Joint target [DEG]", () -> Math.toDegrees(jointTicksToAngle(targetPosJoint)), null);
        builder.addDoubleProperty("Base target [DEG]", () -> Math.toDegrees(baseTicksToAngle(targetPosBase)), null);

        builder.addDoubleProperty("Arm angle calc alpha1", () -> Math.toDegrees(
                ArmKinematics.posToAngles(ArmKinematics.anglesToPos(baseAngle(), jointAngle())).getFirst().base), null);
        builder.addDoubleProperty("Arm angle calc beta1", () -> Math.toDegrees(
                ArmKinematics.posToAngles(ArmKinematics.anglesToPos(baseAngle(), jointAngle())).getFirst().joint),
                null);

        builder.addDoubleProperty("Arm angle calc alpha2", () -> Math.toDegrees(
                ArmKinematics.posToAngles(ArmKinematics.anglesToPos(baseAngle(), jointAngle())).getSecond().base),
                null);
        builder.addDoubleProperty("Arm angle calc beta2", () -> Math.toDegrees(
                ArmKinematics.posToAngles(ArmKinematics.anglesToPos(baseAngle(), jointAngle())).getSecond().joint),
                null);
        builder.addDoubleProperty("Distance sensor [RAW]", this::getGripperArmDistanceSensor, null);
    }
}
