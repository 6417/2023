package frc.robot.subsystems;

import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.ArmCalculator;
import frc.robot.Constants;
import frc.robot.subsystems.base.ArmBase;

public class Arm extends ArmBase {
    private static ArmBase instance = null;

    private class Motors {
        public FridolinsMotor base;
        private FridolinsMotor baseFollower;
        public FridolinsMotor joint;
        private FridolinsMotor jointFollower;

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
        }
    }

    private Motors motors;
    private ArmCalculator calculator;

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

    @Override
    public void init() {
        motors = new Motors();
        calculator = new ArmCalculator(
                new ArmCalculator.ArmState(new ArmCalculator.AnglesSupplier(this::baseAngle, this::jointAngle),
                        Constants.Arm.initialCargo));
    }

    public double baseAngle() {
        return motors.base.getEncoderTicks() / Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.baseGearRatio * Math.PI * 2.0;
    }

    public double jointAngle() {
        return motors.joint.getEncoderTicks() / Constants.Arm.encoderTicksPerMotorRevolution
                * Constants.Arm.jointGearRatio * Math.PI * 2.0;
    }
}
