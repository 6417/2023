package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.utils.LatchBooleanRising;
import frc.fridowpi.utils.LatchedBooleanFalling;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ResetBaseEncodersOnHall extends CommandBase {
    LatchBooleanRising leftHallRising;
    LatchBooleanRising rightHallRising;

    LatchedBooleanFalling leftHallFalling;
    LatchedBooleanFalling rightHallFalling;
    
    Supplier<Boolean> right; 
    Supplier<Boolean> left; 
    
    public ResetBaseEncodersOnHall(Supplier<Boolean> right, Supplier<Boolean> left) {
        this.right = right; 
        this.left = left; 
    }

    @Override
    public void initialize() {
        leftHallRising = new LatchBooleanRising(left.get());
        rightHallRising = new LatchBooleanRising(right.get());

        leftHallFalling = new LatchedBooleanFalling(left.get());
        rightHallFalling = new LatchedBooleanFalling(right.get());
    }

    @Override
    public void execute() {
        leftHallRising.update(left.get());
        rightHallRising.update(right.get());

        leftHallFalling.update(left.get());
        rightHallFalling.update(right.get());

        if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelPositive);
            System.out.printf("Right Falling vel > 0, pos [RAD]: %f\n", Arm.getInstance().baseAngle());
        } else if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelNegative);
            System.out.printf("Right Falling vel < 0, pos [RAD]: %f\n", Arm.getInstance().baseAngle());
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelNegative);
            System.out.printf("Right Rising vel > 0, pos [RAD]: %f\n", Arm.getInstance().baseAngle());
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelPositive);
            System.out.printf("Right Rising vel < 0, pos [RAD]: %f\n", Arm.getInstance().baseAngle());
        }

        // TODO: 
        if (leftHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelPositive);
            System.out.printf("Left Falling vel > 0, pos [DEG]: %f\n", Arm.getInstance().baseAngle());
        } else if (leftHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelNegative);
            System.out.printf("Left Falling vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        } else if (leftHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelNegative);
            System.out.printf("Left Rising vel > 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        } else if (leftHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelPositive);
            System.out.printf("Left Rising vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
