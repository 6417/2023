package frc.fridowpi.motors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import frc.fridowpi.motors.utils.PidValues;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import frc.fridowpi.module.Module;
import frc.fridowpi.module.IModule;

import static java.lang.Math.abs;

public class FridoCanSparkMax extends CANSparkMax implements FridolinsMotor {
    public static List<FridoCanSparkMax> allMotorsWithShitySoftware = new ArrayList<>();

    private IModule moduleProxy = new Module();

    SparkMaxLimitSwitch forwardLimitSwitch;
    SparkMaxLimitSwitch reverseLimitSwitch;
    SparkMaxPIDController pidController;
    ControlType pidControlType;
    RelativeEncoder relativeEncoder;
    double setPoint;
    Optional<Integer> selectedPIDSlotIdx = Optional.empty();

    public FridoCanSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        allMotorsWithShitySoftware.add(this);
    }

    @Override
    public void setPosition(double position) {
        if (this.pidController != null) {
            selectedPIDSlotIdx.ifPresentOrElse(
                    (slotIdx) -> pidController.setReference(position, ControlType.kPosition, slotIdx),
                    () -> pidController.setReference(position, ControlType.kPosition));
            setPoint = position;
            pidControlType = ControlType.kPosition;
        } else {
            try {
                throw new Exception("PID Controller not initialized");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private ControlType pidTypeToSparkControlType(PidType type) {
        switch (type) {
            case velocity:
                return ControlType.kVelocity;
            case voltage:
                return ControlType.kVoltage;
            case position:
                return ControlType.kPosition;
            case smartMotion:
                return ControlType.kSmartMotion;
            case smartVelocity:
                return ControlType.kSmartVelocity;
            default:
                return null;
        }
    }

    @Override
    public void setPidTarget(double value, PidType type) {
        setPoint = value;
        pidControlType = pidTypeToSparkControlType(type);
        pidController.setReference(value, pidControlType);
    }

    @Override
    public void setVelocity(double velocity) {
        if (this.pidController != null) {
            selectedPIDSlotIdx.ifPresentOrElse(
                    (slotIdx) -> pidController.setReference(velocity, ControlType.kVelocity, slotIdx),
                    () -> pidController.setReference(velocity, ControlType.kVelocity));
            setPoint = velocity;
            pidControlType = ControlType.kVelocity;
        } else {
            try {
                throw new Exception("PID Controller not initialized");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private SparkMaxLimitSwitch.Type convertFromFridoLimitSwitchPolarity(FridolinsMotor.LimitSwitchPolarity polarity) {
        switch (polarity) {
            case kNormallyOpen:
                return SparkMaxLimitSwitch.Type.kNormallyOpen;
            case kNormallyClosed:
                return SparkMaxLimitSwitch.Type.kNormallyClosed;
            default:
                return SparkMaxLimitSwitch.Type.kNormallyOpen;
        }
    }

    private SparkMaxLimitSwitch getForwardLimitSwitchInstance() {
        return forwardLimitSwitch;
    }

    private SparkMaxLimitSwitch getForwardLimitSwitchInstance(FridolinsMotor.LimitSwitchPolarity polarity) {
        if (forwardLimitSwitch == null) {
            forwardLimitSwitch = super.getForwardLimitSwitch(convertFromFridoLimitSwitchPolarity(polarity));
        }
        return forwardLimitSwitch;
    }

    private SparkMaxLimitSwitch getReverseLimitSwitchInstance() {
        return reverseLimitSwitch;
    }

    private SparkMaxLimitSwitch getReverselimitSwitchInstance(FridolinsMotor.LimitSwitchPolarity polarity) {
        if (reverseLimitSwitch == null) {
            reverseLimitSwitch = super.getReverseLimitSwitch(convertFromFridoLimitSwitchPolarity(polarity));
        }
        return reverseLimitSwitch;
    }

    @Override
    public void enableForwardLimitSwitch(FridolinsMotor.LimitSwitchPolarity polarity, boolean enable) {
        getForwardLimitSwitchInstance(polarity).enableLimitSwitch(enable);
    }

    @Override
    public void enableReverseLimitSwitch(FridolinsMotor.LimitSwitchPolarity polarity, boolean enable) {
        getReverselimitSwitchInstance(polarity).enableLimitSwitch(enable);
    }

    @Override
    public boolean isForwardLimitSwitchActive() {
        return getForwardLimitSwitchInstance().isPressed();
    }

    @Override
    public boolean isReverseLimitSwitchActive() {
        return getReverseLimitSwitchInstance().isPressed();
    }

    private CANSparkMax.IdleMode convertFromFridoIdleModeType(FridolinsMotor.IdleMode idleModeType) {
        switch (idleModeType) {
            case kBrake:
                return CANSparkMax.IdleMode.kBrake;
            case kCoast:
                return CANSparkMax.IdleMode.kCoast;
            default:
                return CANSparkMax.IdleMode.kBrake;
        }
    }

    @Override
    public void setIdleMode(FridolinsMotor.IdleMode type) {
        this.setIdleMode(convertFromFridoIdleModeType(type));
    }

    private boolean convertFromFridoDirectionsType(FridolinsMotor.DirectionType direction) {
        switch (direction) {
            case followMaster:
                return false;
            case invertMaster:
                return true;
            default:
                return false;
        }
    }

    @Override
    public void follow(FridolinsMotor master, FridolinsMotor.DirectionType direction) {
        if (master instanceof FridoCanSparkMax) {
            super.follow((FridoCanSparkMax) master, convertFromFridoDirectionsType(direction));
        }
    }

    @Override
    public void stopMotor() {
        super.stopMotor();
    }

    @Override
    public void setInverted(boolean inverted) {
        super.setInverted(inverted);
    }

    @Override
    public void setEncoderDirection(boolean inverted) {
        if (relativeEncoder == null)
            throw new Error("configEncoder must be called before using the setEncoderDirection function");
        this.relativeEncoder.setInverted(inverted);
    }

    @Override
    public void setEncoderPosition(double position) {
        this.relativeEncoder.setPosition(position);
    }

    @Override
    public double getEncoderTicks() {
        return this.relativeEncoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return this.relativeEncoder.getVelocity();
    }

    @Override
    public void factoryDefault() {
        super.restoreFactoryDefaults();
    }

    private Type convertEncoderType(FridoFeedBackDevice device) {
        switch (device) {
            case kAlternative:
                return Type.kHallSensor;
            case kRelative:
                return Type.kQuadrature;
            default:
                return Type.kHallSensor;
        }
    }

    @Override
    public void configEncoder(FridoFeedBackDevice device, int countsPerRev) {
        if (device == FridoFeedBackDevice.kBuildin) {
            this.relativeEncoder = super.getEncoder();
        } else {
            this.relativeEncoder = super.getEncoder(convertEncoderType(device), countsPerRev);
            this.relativeEncoder.setPositionConversionFactor(countsPerRev);
        }
    }

    public void selectBuiltinFeedbackSensor() {
        this.relativeEncoder = super.getEncoder();
        super.getEncoder().setPositionConversionFactor(0);
    }

    @Override
    public void configOpenLoopRamp(double rate) {
        super.setOpenLoopRampRate(rate);
    }

    @Override
    public double getPidTarget() {
        return setPoint;
    }

    private void setMotionMagicParametersIfNecessary(PidValues pidValues) {
//        if (pidValues.cruiseVelocity.isPresent() || pidValues.acceleration.isPresent())
//            assert pidValues.slotIdX.isPresent() : "To set cruiseVelocity or acceleration slotIdx needs to be set";
        pidValues.cruiseVelocity.ifPresent((cruiseVelocity) -> this.pidController
                .setSmartMotionMaxVelocity(cruiseVelocity, pidValues.slotIdX.orElse(selectedPIDSlotIdx.orElse(0))));
        pidValues.acceleration.ifPresent(
                (acceleration) -> this.pidController.setSmartMotionMaxAccel(acceleration, pidValues.slotIdX.orElse(selectedPIDSlotIdx.orElse(0))));
    }

    private Optional<Double> tolerance = Optional.empty();

    @Override
    public void setPID(PidValues pidValues) {
        this.pidController = super.getPIDController();
        tolerance = pidValues.tolerance;
        pidValues.slotIdX.ifPresentOrElse((slotIdx) -> setPIDWithSlotIdx(pidValues),
                () -> setPIDWithOutSlotIdx(pidValues));

        setMotionMagicParametersIfNecessary(pidValues);
    }

    @Override
    public boolean pidAtTarget() {
        switch (pidControlType) {
            case kDutyCycle:
            case kVelocity:
            case kSmartVelocity:
                return abs(setPoint - getEncoderVelocity()) < tolerance.orElse(0.0);

            case kVoltage:
                return abs(super.getBusVoltage() - setPoint) < tolerance.orElse(0.0);

            case kPosition:
            case kSmartMotion:
                return abs(setPoint - getEncoderTicks()) < tolerance.orElse(0.0);

            case kCurrent:
                return abs(getOutputCurrent() - setPoint) < tolerance.orElse(0.0);
            default:
                return true;
        }
    }

    private void setPIDWithOutSlotIdx(PidValues pidValues) {
        this.pidController.setP(pidValues.kP);
        this.pidController.setI(pidValues.kI);
        this.pidController.setD(pidValues.kD);
        pidValues.kF.ifPresent((kF) -> this.pidController.setFF(kF));
        tolerance = pidValues.tolerance;
        this.pidController.setOutputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward);
    }

    private void setPIDWithSlotIdx(PidValues pidValues) {
        this.pidController.setP(pidValues.kP, pidValues.slotIdX.get());
        this.pidController.setI(pidValues.kI, pidValues.slotIdX.get());
        this.pidController.setD(pidValues.kD, pidValues.slotIdX.get());
        pidValues.kF.ifPresent((kF) -> this.pidController.setFF(kF, pidValues.slotIdX.get()));
        this.pidController.setOutputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward,
                pidValues.slotIdX.get());
    }

    @Override
    public void selectPidSlot(int slotIndex) {
        selectedPIDSlotIdx = Optional.of(slotIndex);
    }

    @Override
    public Collection<IModule> getAllSubModules() {
        return moduleProxy.getAllSubModules();
    }

    @Override
    public Collection<IModule> getSubModules() {
        return moduleProxy.getSubModules();
    }

    @Override
    public void registerSubmodule(IModule... subModule) {
        moduleProxy.registerSubmodule(subModule);
    }

    private boolean initialized = false;

    @Override
    public void init() {
        initialized = true;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }
}