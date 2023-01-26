package frc.robot.subsystems;

import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;
import frc.robot.subsystems.base.ArmBase;

public class Arm extends ArmBase {
    private static ArmBase instance = null;

    private class Motors {
        public FridolinsMotor base;
        public FridolinsMotor baseFollower;
        public FridolinsMotor joint;
        public FridolinsMotor jointFollower;

        public Motors() {
            // TODO: Initialize motors
        }
    }

    private Motors motors;

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
    }
}
