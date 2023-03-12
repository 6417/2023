package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.utils.LatchedBooleanRising;
import frc.fridowpi.utils.LatchedBooleanFalling;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ResetBaseEncodersOnHall extends CommandBase {
    LatchedBooleanRising leftHallRising;
    LatchedBooleanRising rightHallRising;

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
        leftHallRising = new LatchedBooleanRising(left.get());
        rightHallRising = new LatchedBooleanRising(right.get());

        leftHallFalling = new LatchedBooleanFalling(left.get());
        rightHallFalling = new LatchedBooleanFalling(right.get());
    }

    @Override
    public void execute() {
        leftHallRising.update(left.get());
        rightHallRising.update(right.get());

        leftHallFalling.update(left.get());
        rightHallFalling.update(right.get());

// Right Falling vel < 0, pos [DEG]: 92.515869 
// Right Rising vel < 0, pos [DEG]: 84.469482 
// Right Falling vel > 0, pos [DEG]: 85.904297
// Right Rising vel > 0, pos [DEG]: 93.795776 
// Right Falling vel < 0, pos [DEG]: 92.514771 
// Right Rising vel < 0, pos [DEG]: 84.471680
        if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightFallingEncoderPosHallVelPositive);
        } else if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightFallingEncoderPosHallVelNegative);
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightRisingEncoderPosHallVelPositive);
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightRisingEncoderPosHallVelNegative);
        }

        // TODO: 
        // if (leftHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
        //     // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelPositive);
        //     System.out.printf("Left Falling vel > 0, pos [DEG]: %f\n", Arm.getInstance().baseAngle());
        // } else if (leftHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
        //     // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelNegative);
        //     System.out.printf("Left Falling vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        // } else if (leftHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
        //     // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelNegative);
        //     System.out.printf("Left Rising vel > 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        // } else if (leftHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
        //     // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelPositive);
        //     System.out.printf("Left Rising vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        // }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
