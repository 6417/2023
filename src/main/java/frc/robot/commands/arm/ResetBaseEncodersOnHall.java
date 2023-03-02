package frc.robot.commands.arm;

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

    @Override
    public void initialize() {
        leftHallRising = new LatchBooleanRising(Arm.getInstance().isBaseLeftHallActive());
        rightHallRising = new LatchBooleanRising(Arm.getInstance().isBaseRightHallActive());

        leftHallFalling = new LatchedBooleanFalling(Arm.getInstance().isBaseLeftHallActive());
        rightHallFalling = new LatchedBooleanFalling(Arm.getInstance().isBaseRightHallActive());
    }

    @Override
    public void execute() {
        leftHallRising.update(Arm.getInstance().isBaseLeftHallActive());
        rightHallRising.update(Arm.getInstance().isBaseRightHallActive());

        leftHallFalling.update(Arm.getInstance().isBaseLeftHallActive());
        rightHallFalling.update(Arm.getInstance().isBaseRightHallActive());

        if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelPositive);
            System.out.printf("Right Falling vel > 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        } else if (rightHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelNegative);
            System.out.printf("Right Falling vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelNegative);
            System.out.printf("Right Rising vel > 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        } else if (rightHallRising.get() && Arm.getInstance().getBaseEncoderVelocity() < 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseRightEncoderPosHallVelPositive);
            System.out.printf("Right Rising vel < 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
        }

        if (leftHallFalling.get() && Arm.getInstance().getBaseEncoderVelocity() > 0) {
            // Arm.getInstance().setEncoderTicksBase(Constants.Arm.baseLeftEncoderPosHallVelPositive);
            System.out.printf("Left Falling vel > 0, pos [DEG]: %f", Arm.getInstance().baseAngle());
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
