package frc.robot.subsystems.base;

import frc.fridowpi.module.Module;
import frc.robot.ArmModel;
import frc.robot.ArmModel.Cargo;

public class ArmBase extends Module {
    public double baseAngle() {
        return Math.PI / 2.0;
    }

    public double jointAngle() {
        return 0.0;
    }
    
    public double getJointTargetAngle() {
        return jointAngle();
    }

    public double getBaseTargetAngle() {
        return baseAngle();
    }
    
    public ArmModel getModel() {
        return new ArmModel(new ArmModel.ArmStateSupplier(this::baseAngle, this::jointAngle, () -> 0.0, () -> 0.0), Cargo.None);
    }

    public void setPercentBase(double percent) {

    }

    public void setPercentJoint(double percent) {

    }

    public void setEncoderTicksBase(double ticks) {

    }

    public void setEncoderTicksJoint(double ticks) {

    }

    public boolean getBaseLimitSwitchFwd() {
        return false;
    }

    public boolean getBaseLimitSwitchRev() {
        return false;
    }
    
    public void baseGotoAngle(double angle) {

    }

    public void jointGotoAngle(double angle) {

    }
    
    public void setBasePercent(double percent) {

    }

    public void setJointPercent(double percent) {

    }
    
    public boolean baseIsAtTarget() {
        return true;
    }
    
    public void stop() {
        
    }
    
    public void hold() {
        
    }
    
    public boolean isZeroed() {
        return true;
    }

    public boolean isBaseRightHallActive() {
        return false;
    }

    public boolean isBaseLeftHallActive() {
        return false;
    }
    
    public double getBaseEncoderVelocity() {
        return 0.0;
    }
    
    public boolean isBaseAtTarget() {
        return true;
    }

    public boolean isJointAtTarget() {
        return true;
    }
    
    public boolean isArmAtTarget() {
        return isBaseAtTarget() && isJointAtTarget();
    }
}
