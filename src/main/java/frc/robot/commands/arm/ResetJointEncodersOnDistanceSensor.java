package frc.robot.commands.arm;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.utils.LatchedBooleanRising;
import frc.fridowpi.utils.LatchedBooleanFalling;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ResetJointEncodersOnDistanceSensor extends CommandBase {
    LatchedBooleanFalling falling;
    LatchedBooleanRising rising;
    boolean velSign = false;
    boolean updatedForVelSign = false;

    private boolean sensorOverThreshold() {
        return Arm.getInstance().getGripperArmDistanceSensor() < Constants.Arm.gripperArmDistanceSensorThreshold;
    }

    @Override
    public void initialize() {
        falling = new LatchedBooleanFalling(sensorOverThreshold());
        rising = new LatchedBooleanRising(sensorOverThreshold());
    }

    @Override
    public void execute() {

        velSign = !(velSign && (velSign ? Arm.getInstance().getJointEncoderVelocity() < 0
                : Arm.getInstance().getJointEncoderVelocity() > 0));

        if (falling.updateAndGet(sensorOverThreshold()) && Arm.getInstance().getJointEncoderVelocity() < 0) {
            System.out.println("Joint distance sensor FALLING, joint angle [DEG]: "
                    + Math.toDegrees(Arm.getInstance().jointAngle()));

            // if (!Arm.getInstance().isJointZeroed()) {
            //     Arm.getInstance().setEncoderTicksJoint(Arm.jointAngleToTicks(Math.toRadians(9.64)));
            // }
        }

        if (rising.updateAndGet(sensorOverThreshold()) && Arm.getInstance().getJointEncoderVelocity() > 0) {
            System.out.println("Joint distance sensor RISING, joint angle [DEG]: "
                    + Math.toDegrees(Arm.getInstance().jointAngle()));
            // if (!Arm.getInstance().isJointZeroed()) {
            //     Arm.getInstance().setEncoderTicksJoint(Arm.jointAngleToTicks(Math.toRadians(11.5)));
            // }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
