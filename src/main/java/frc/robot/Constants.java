// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.robot.ArmModel.Cargo;

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
        public static double baseGearRatio = 1.0 / 160.0; 
        public static double jointGearRatio = 1.0 / 30.0;
        public static double encoderTicksPerMotorRevolution = 2048.0;

        public static double torqueToAmpsProportionality = 1 / 0.02076;

        public static ArmModel.Cargo initialCargo = Cargo.None;
        public static boolean enabled = true;

        public static class PhysicalProperties {
            public double centerOfMass;
            public double mass;
            public double length;
			public double momentOfInertia;

            public PhysicalProperties(double centerOfMass, double mass, double length, double momentOfInertia) {
                this.centerOfMass = centerOfMass;
                this.mass = mass;
                this.length = length;
                this.momentOfInertia = momentOfInertia;
            }
        }

        public static final LimitSwitchPolarity limitSwitchPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final PhysicalProperties baseArm = new PhysicalProperties(0.782, 7.083, 1.10, 2747698.280 / 1e6);
        public static final PhysicalProperties gripperArm = new PhysicalProperties(0.55, 2.153, 0.800, 6538.368 / 1e6); // TODO: check value of moment of inertia

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
