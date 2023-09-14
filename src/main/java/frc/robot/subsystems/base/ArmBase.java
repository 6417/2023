package frc.robot.subsystems.base;

import frc.fridowpi.module.Module;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmKinematics;
import frc.robot.ArmModel;
import frc.robot.ArmPathGenerator;
import frc.robot.ArmModel.Cargo;
import frc.robot.ArmPathGenerator.RobotOrientation;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.subsystems.Arm;

public class ArmBase extends Module {
    public boolean xDirInverted = false; 
    
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
    
    public boolean isPosValid(Vector2 pos) {
        return true;
    }

    public ArmModel getModel() {
        return new ArmModel(new ArmModel.ArmStateSupplier(this::baseAngle, this::jointAngle, () -> 0.0, () -> 0.0),
                Cargo.None);
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

    public void baseGotoAngleNoZeroedCheckOfJoint(double angle) {

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

    public double getBaseEncoderVelocity() {
        return 0.0;
    }

    public boolean isBaseAtTargetDriveThrough() {
        return true;
    }

    public boolean isJointAtTargetDriveThrough() {
        return true;
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

    public boolean isArmAtTargetDriveThrough() {
        return isBaseAtTargetDriveThrough() && isJointAtTargetDriveThrough();
    }

    public Vector2 getPos() {
        return ArmKinematics.anglesToPos(baseAngle(), jointAngle());
    }
    
    public void setManualControlMode(Arm.ManualControlMode mode) {

    }
    
    public ArmPathGenerator.RobotOrientation getRobotOrientation() {
        return RobotOrientation.FORWARD;
    }
    
    public ArmPathGenerator.RobotPos getRobotPos() {
        return RobotPos.FIELD;
    }

    public void setRobotOrientation(ArmPathGenerator.RobotOrientation orientation) {

    }
    
    public void setRobotPos(ArmPathGenerator.RobotPos pos) {
    }
    
    public double getGripperArmDistanceSensor() {
        return 0.0;
    }

    public boolean isJointZeroed() {
        return true;
    }

    public boolean isBaseZeroed() {
        return true;
    }
    
    public double getJointEncoderVelocity() {
        return 0.0;
    }

    public void enableBreakModeJoint() {

    }

    public void disableBreakModeJoint() {

    }

    
    public int getLastValidQuadrant() {
        return -1;
    }
    
    public void setJointPidTolerance(double ticks) {

    }

    public void setBasePidTolerance(double ticks) {

    }
}
