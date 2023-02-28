// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.robot.subsystems.drive.Drive.SteerMode;
import frc.fridowpi.joystick.IJoystickButtonId;

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
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class Joystick {
        public static final IJoystickId accelerator = () -> 0;
        public static final IJoystickId steeringWheel = () -> 1;
    }

    public final static class Drive {
        public static final boolean enabled = true;
        public static final int movingAveragePrecision = 20;

        public static final class Defaults {
            public static final SteerMode steerMode = SteerMode.CARLIKE;
            public static final boolean steerWithJoystick = true;
        }
        
        public static class Motors {
            public static final int BACKLEFT = 13;
            public static final int BACKRIGHT = 12;
            public static final int FRONTLEFT = 11;
            public static final int FRONTRIGHT = 10;
        }

        public static final class Odometry {
            public static final double wheelPerimeter = 0.47;
            // The transmission denotes how many revolution the motor makes compared to the wheel
            public static final double transmission = 10.71;
            public static final int encoderResolution = 2048;

            public static final double encoderToMetersConversion = 1 / ((1 / wheelPerimeter) * transmission * encoderResolution);
            public static final double trackWidthMeters = 0.5;
        }

        public static final class PathWeaver {
            public static final double ksMeters = 0.12091;
            public static final double kvMetersPerSecoond = 2.3501;
            public static final double ka = 0.21997;

            public static final double kMaxSpeed = 0;
            public static final double kMaxAcceleration = 0;
            public static final double kMaxCentripetalAcceleration = 0;

            public static final double kRamseteB = 0;
            public static final double kRamseteZeta = 0;

            public static final double kP = 0.36205;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static class Autonomous {
            public static final double recordingCooldownSeconds = 0.1;
            public static final double velocityThresholdStart = 0.01;
            public static final double velocityThresholdEnd = 0.02;
            public static final double positionCorrection = 0;
        }

        public static final class Brake {
            public static final class FridoDoubleSolenoid {
                public static final int rightIdLower = 0;
                public static final int rightIdHigher = 1;

                public static final int leftIdLower = 0;
                public static final int leftIdHigher = 1;
            }
        }

        public static final class ButtonIds {
            public static final IJoystickButtonId driveForward = LogitechExtreme._11;
            public static final IJoystickButtonId driveBackward = LogitechExtreme._12;
            public static final IJoystickButtonId steerModeCarlike = LogitechExtreme._7;
            public static final IJoystickButtonId steerModeBidirectional = LogitechExtreme._9;

            public static final IJoystickButtonId activateBrake = LogitechExtreme._4;

            public static final IJoystickButtonId activateBalancing = LogitechExtreme._5;
        }
        

        
    }
}
