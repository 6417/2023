package frc.robot;

import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.robot.subsystems.drive.Drive.SteerMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.fridowpi.joystick.IJoystickButtonId;

import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.utils.Vector2;
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
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class Joysticks {
        public static final IJoystickId accelerator = () -> 0;
        public static final IJoystickId steeringWheel = () -> 1;
        public static final IJoystickId armJoystick = () -> 2;
        public static final double armJoystickDeadZone = 0.1;
    }

    public final static class Drive {
        public static final boolean enabled = true;
        public static final int movingAveragePrecision = 20;

        public static final class Defaults {
            public static final SteerMode steerMode = SteerMode.CARLIKE;
            public static final boolean steerWithJoystick = false;
        }

        public static class Motors {
            public static final int BACKLEFT = 13;
            public static final int BACKRIGHT = 12;
            public static final int FRONTLEFT = 11;
            public static final int FRONTRIGHT = 10;
        }

        public static final class Odometry {
            public static final double wheelPerimeter = 0.47;
            // The transmission denotes how many revolution the motor makes compared to the
            // wheel
            public static final double transmission = 10.71;
            public static final int encoderResolution = 2048;

            public static final double encoderToMetersConversion = 1
                    / ((1 / wheelPerimeter) * transmission * encoderResolution);
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

    public static final double gravity = 9.81; // m/s^2

    public static class Arm {
        public static class PhysicalProperties {
            public final double centerOfMass;
            public final double mass;
            public final double length;
            public final double momentOfInertia;

            public PhysicalProperties(double centerOfMass, double mass, double length, double momentOfInertia) {
                this.centerOfMass = centerOfMass;
                this.mass = mass;
                this.length = length;
                this.momentOfInertia = momentOfInertia;
            }
        }

        public static class PIDValues {
            public double p;
            public double i;
            public double d;
            public double iZone;
            public double maxIntegralAccum;
            public double allowableError;

            public PIDValues(
                    double p,
                    double i,
                    double d,
                    double iZone,
                    double maxIntegralAccum,
                    double allowableError) {
                this.p = p;
                this.i = i;
                this.d = d;
                this.iZone = iZone;
                this.maxIntegralAccum = maxIntegralAccum;
                this.allowableError = allowableError;
            }
        }

        public static class MotionMagic {
            public double cruiseVel;
            public double accel;
            public int curveStrength;

            public MotionMagic(
                    double cruiseVel,
                    double accel,
                    int curveStrength) {
                this.cruiseVel = cruiseVel;
                this.accel = accel;
                this.curveStrength = curveStrength;
            }
        }

        public static final double baseGearRatio = 1.0 / 160.0;
        public static final double jointGearRatio = 1.0 / 30.0;
        public static final double encoderTicksPerMotorRevolution = 2048.0;
        

        public static final double torqueToAmpsProportionality = 1 / 0.02076;

        public static final double jointMaxAmps = torqueToAmpsProportionality * 1.5;

        public static final ArmModel.Cargo initialCargo = Cargo.None;
        public static final boolean enabled = true;
        public static final double jointCenterOfMassAngleOffset = Math.toRadians(-2.955);

        public static final PIDValues basePid = new PIDValues(0.775, 2e-5, 0.01, 2000, 0, 10);
        public static final MotionMagic baseMotionMagic = new MotionMagic(9500, 5000, 1); // TODO: set vel back to 9500

        public static final LimitSwitchPolarity limitSwitchPolarity = LimitSwitchPolarity.kNormallyClosed;
        public static final int dioIdHallBaseRight = 9;
        public static final int dioIdHallBaseLeft = 8;
        public static final PhysicalProperties baseArm = new PhysicalProperties(0.782, 7.083, 1.10, 2747698.280 / 1e6);

        public static final PhysicalProperties gripperArm = new PhysicalProperties(0.55,
                2.153, 0.840, 6538.368 / 1e6);

        public static final DirectionType baseFollowType = DirectionType.followMaster;
        public static final DirectionType jointFollowType = DirectionType.invertMaster;

        public static final double baseStatorCurrentLimit = torqueToAmpsProportionality * 1.5;
        public static final double jointStatorCurrentLimit = torqueToAmpsProportionality * 1.5;

        public static final double baseEncoderPosFwdLimitSwitch = 150.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0;
        public static final double baseEncoderPosRevLimitSwitch = 30.0 / 360.0 / Constants.Arm.baseGearRatio * 2048.0;

        public static final Vector2 armPathGenForwardOffset = new Vector2(0.26, 0.0);
        public static final Vector2 armPathGenReverseOffset = new Vector2(0.61, 0.0);

// Right Falling vel < 0, pos [DEG]: 92.515869 
// Right Rising vel < 0, pos [DEG]: 84.469482 
// Right Falling vel > 0, pos [DEG]: 85.904297
// Right Rising vel > 0, pos [DEG]: 93.795776 
//
// Right Falling vel < 0, pos [DEG]: 92.514771 
// Right Rising vel < 0, pos [DEG]: 84.471680
        public static final double baseRightFallingEncoderPosHallVelNegative = frc.robot.subsystems.Arm.baseAngleToTicks(Math.toRadians(92.515869));
        public static final double baseRightRisingEncoderPosHallVelNegative = frc.robot.subsystems.Arm.baseAngleToTicks(Math.toRadians(84.469482));
        public static final double baseRightFallingEncoderPosHallVelPositive = frc.robot.subsystems.Arm.baseAngleToTicks(Math.toRadians(85.90429));
        public static final double baseRightRisingEncoderPosHallVelPositive  = frc.robot.subsystems.Arm.baseAngleToTicks(Math.toRadians(93.795776));
        
        public static final int gripperArmDistanceSensorAnalogInputPin = 0;
        public static final double gripperArmDistanceSensorThreshold = 1.2;
        
        
        public static final double baseLeftEncoderPosHallVelPositive = 0.0;
        public static final double baseLeftEncoderPosHallVelNegative = 0.0;

        public static final double baseAllowableErrorDriveThrough = 300;
        public static final double jointAllowableErrorDriveThrough = 1000;

        public static final double baseAllowableError = 300;
        public static final double jointAllowableError = 800;

        public static final double baseDefaultManualMaxPercent = 0.3;
        public static final double jointDefaultManualMaxPercent = 0.3;

        public static class Ids {
            public static final int baseMotor = 20;
            public static final int baseFollowerMotor = 21;

            public static final int jointMotor = 22;
            public static final int jointFollowerMotor = 23;
        }
    }

    public static class Gripper {
        public static final boolean enabled = true;

        public static class ButtonIds {
            public static final IJoystickButtonId openClose = Logitech.a;
        }

        public static final int idHigher = 3;
        public static final int idLower = 2;
        public static final PneumaticsModuleType type = PneumaticsModuleType.CTREPCM;
        public static final int compressorId = 0;
        public static final Value gripperClosed = Value.kReverse;
        public static final Value gripperOpen = Value.kForward;
    }
}
