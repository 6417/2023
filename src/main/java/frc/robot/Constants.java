// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.robot.ArmCalculator.Cargo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double gravity = 9.81; // m/s^2

    public static class Arm {
        public static double baseGearRatio = 1.0 / 60.0;
        public static double jointGearRatio = 1.0 / 30.0;
        public static double encoderTicksPerMotorRevolution = 2048.0;

        public static ArmCalculator.Cargo initialCargo = Cargo.None;
        public static boolean enabled = true;

        public static class PhysicalProperties {
            public double centerOfMass;
            public double mass;
            public double length;

            public PhysicalProperties(double centerOfMass, double mass, double length) {
                this.centerOfMass = centerOfMass;
                this.mass = mass;
            }
        }

        public static final PhysicalProperties baseArm = new PhysicalProperties(0.0, 0.0, 110.0);
        public static final PhysicalProperties gripperArm = new PhysicalProperties(0.0, 0.0, 80.0);

        public static final DirectionType baseFollowType = DirectionType.invertMaster;
        public static final DirectionType jointFollowType = DirectionType.invertMaster;

        public static class Ids {
            public static final int baseMotor = 20;
            public static final int baseFollowerMotor = 21;

            public static final int jointMotor = 22;
            public static final int jointFollowerMotor = 23;
        }
    }
}
