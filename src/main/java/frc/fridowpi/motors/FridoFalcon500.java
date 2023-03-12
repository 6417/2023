package frc.fridowpi.motors;

import java.util.Collection;
import java.util.Optional;

import frc.fridowpi.motors.utils.PidValues;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.fridowpi.module.IModule;
import frc.fridowpi.module.Module;

import static java.lang.Math.abs;

public class FridoFalcon500 extends TalonFX implements FridolinsMotor {
    Module moduleProxy = new Module();
    Optional<Integer> pidSlotIdx;

    public FridoFalcon500(int deviceNumber) {
        super(deviceNumber);
    }

    public FeedbackDevice convertFromTalonFXFeedbackDevice(FridoFeedBackDevice device) {
        switch (device) {
            case kRelative:
                return FeedbackDevice.QuadEncoder;
            case kBuildin:
                return FeedbackDevice.IntegratedSensor;
            default:
                throw new Error("Feedbackdevice not avaible");
        }
    }

    @Override
    public void configEncoder(FridoFeedBackDevice device, int countsPerRev) {
        super.configSelectedFeedbackSensor(convertFromTalonFXFeedbackDevice(device));
    }

    @Override
    public void setEncoderDirection(boolean inverted) {
        super.setSensorPhase(inverted);
    }

    @Override
    public void setEncoderPosition(double position) {
        super.setSelectedSensorPosition((int) position);
    }

    @Override
    public double getEncoderTicks() {
        return super.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return super.getSelectedSensorVelocity();
    }

    private LimitSwitchNormal convertFromFridoLimitSwitchPolarity(LimitSwitchPolarity polarity) {
        switch (polarity) {
            case kDisabled:
                return LimitSwitchNormal.Disabled;
            case kNormallyClosed:
                return LimitSwitchNormal.NormallyClosed;
            case kNormallyOpen:
                return LimitSwitchNormal.NormallyOpen;
            default:
                return LimitSwitchNormal.NormallyOpen;
        }
    }

    @Override
    public void enableForwardLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        if (!enable) {
            polarity = LimitSwitchPolarity.kDisabled;
        }
        super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, convertFromFridoLimitSwitchPolarity(polarity));
    }

    @Override
    public void enableReverseLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        if (!enable) {
            polarity = LimitSwitchPolarity.kDisabled;
        }
        super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, convertFromFridoLimitSwitchPolarity(polarity));
    }

    @Override
    public boolean isForwardLimitSwitchActive() {
        return super.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    @Override
    public boolean isReverseLimitSwitchActive() {
        return super.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void setVelocity(double velocity) {
        super.set(TalonFXControlMode.Velocity, velocity);
    }

    @Override
    public void setPosition(double position) {
        super.set(TalonFXControlMode.Position, position);
    }

    @Override
    public void setPidTarget(double value, PidType type) {
        // TODO: implement this function
        assert false : "Not implemented";
    }

    @Override
    public void selectPidSlot(int slotIndex) {
        this.pidSlotIdx = Optional.of(slotIndex);
    }

    @Override
    public void set(double speed) {
        super.set(TalonFXControlMode.PercentOutput, speed);
    }

    @Override
    public double get() {
        return super.getMotorOutputPercent();
    }

    @Override
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return super.getInverted();
    }

    @Override
    public void disable() {
        super.set(TalonFXControlMode.Disabled, 0);
    }

    @Override
    public void stopMotor() {
        super.set(TalonFXControlMode.PercentOutput, 0);
    }

    @Override
    public void configOpenLoopRamp(double rate) {
        super.configOpenloopRamp(rate);
    }

    @Override
    public double getPidTarget() {
        throw new Error("Not implemented");
    }

    public Optional<Double> tolerance;

    @Override
    public void setPID(PidValues pidValues) {
        if (!pidSlotIdx.isPresent()) {
            pidSlotIdx = Optional.of(0);
        }
        super.config_kP(pidSlotIdx.get(), pidValues.kP);
        super.config_kI(pidSlotIdx.get(), pidValues.kI);
        super.config_kD(pidSlotIdx.get(), pidValues.kD);
        pidValues.kF.ifPresent((kF) -> {
            super.config_kF(pidSlotIdx.get(), pidValues.kF.get());
        });
        pidValues.tolerance.ifPresent((tolerance) ->
                super.configAllowableClosedloopError(pidSlotIdx.get(), tolerance)
        );
        tolerance = pidValues.tolerance;
    }

    @Override
    public boolean pidAtTarget() {
        return abs(super.getClosedLoopError() - tolerance.orElse(0.0)) < tolerance.orElse(0.0);
    }

    private NeutralMode convertFromFridoIdleMode(IdleMode mode) {
        switch (mode) {
            case kBrake:
                return NeutralMode.Brake;
            case kCoast:
                return NeutralMode.Coast;
            default:
                return NeutralMode.Coast;
        }
    }

    @Override
    public void setIdleMode(IdleMode type) {
        super.setNeutralMode(convertFromFridoIdleMode(type));
    }

    @Override
    public void follow(FridolinsMotor master, DirectionType direction) {
        if (master instanceof IMotorController) {
            super.follow((IMotorController) master, FollowerType.PercentOutput);
            if (direction == DirectionType.invertMaster) 
                super.setInverted(InvertType.OpposeMaster);
            else if (direction == DirectionType.followMaster)
                super.setInverted(InvertType.FollowMaster);
        }
        else
            throw new Error("Can only follow 'com.ctre.phoenix.motorcontrol.IMotorController' motors");
    }

    @Override
    public void factoryDefault() {
        super.configFactoryDefault();
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
