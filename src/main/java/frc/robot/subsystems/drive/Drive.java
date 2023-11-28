// Refactoring ideas:
//  organizing methods && fields
//  check @Override
//  sync DriveBase

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.command.Command;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.joysticks.LogitechExtreme;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Constants;
import frc.robot.commands.balance.PIDBalanceCommand;
import frc.robot.commands.driveCommands.BrakeCommand;
import frc.robot.commands.driveCommands.ReverseDrivingDirection;
import frc.robot.commands.driveCommands.SetSteerMode;
import frc.robot.subsystems.base.DriveBase;

public class Drive extends DriveBase {
    private static DriveBase instance;

    private Motors motors;

    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrive tankDrive;
    private LinearFilter driveFilter;

    private double driveDirection = -1;
    private static final double defaultDriveSpeed = 1;
    private static final double fastSpeed = 0.8;
    private static final double slowSpeed = 0.15;
    private double speed = defaultDriveSpeed;
    private double motorspeed;

    private double steerDirection = -1;
    private double steeringWheelSensibility = 1;
    private SteerMode steerMode = Constants.Drive.Defaults.steerMode;

    private PIDController rightVelocityController;
    private PIDController leftVelocityController;

    private FridoDoubleSolenoid brakeSolenoid;

    private ArrayList<Float> balancevalues = new ArrayList<>();

    private boolean steerWithJoystick = Constants.Drive.Defaults.steerWithJoystick;

    private double joystickSteeringSensibility = 1.0;


    // Constrains
    private SimpleMotorFeedforward motorFeedForward;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private DifferentialDriveKinematicsConstraint kinematicsConstraint;
    private CentripetalAccelerationConstraint centripetalAccelerationConstraint;
    // Functions as a requirement no functionality
    private Subsystem speedRequirement = new Subsystem() {

    };

    // Config
    private TrajectoryConfig trajectoryConfig;

    @Override
    public TrajectoryConfig getTrajectoryConfig() {
        return trajectoryConfig;
    }

    public static enum SteerMode {
        CARLIKE, BIDIRECTIONAL
    }

    private Drive() {
        motors = new Motors();

        odometry = new DifferentialDriveOdometry(
                new Rotation2d(0),
                0.0, 0.0,
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        kinematics = new DifferentialDriveKinematics(
                Constants.Drive.Odometry.trackWidthMeters);

        driveFilter = LinearFilter.movingAverage(Constants.Drive.movingAveragePrecision);

        FridoNavx.getInstance().init();

    }

    public static DriveBase getInstance() {
        if (instance != null) {
            return instance;
        }
        // If enabled, return a Drive object, else an empty DriveBase
        instance = Constants.Drive.enabled ? new Drive() : new DriveBase();
        return instance;
    }

    @Override
    public void init() {
        // setDefaultCommand(new DriveCommand());

        motors.init();

        tankDrive = new DifferentialDrive(motors.left(), motors.right());
        tankDrive.setDeadband(0.0);

        resetSensors();
        resetVariables();

        configSimpleMotorFeedforward();
        configConstrains();
        configTrajectoryConfig();

        brakeSolenoid = new FridoDoubleSolenoid(0, 1);
        brakeSolenoid.init();

        rightVelocityController = new PIDController(Constants.Drive.PathWeaver.kP, Constants.Drive.PathWeaver.kI,
                Constants.Drive.PathWeaver.kD);
        leftVelocityController = new PIDController(Constants.Drive.PathWeaver.kP, Constants.Drive.PathWeaver.kI,
                Constants.Drive.PathWeaver.kD);

        System.out.println("Drive init completed");
    }

    @Override
    public void reset() {
        resetSensors();
        resetVariables();
    }

    private void resetVariables() {
    }

    @Override
    public void driveJoystick(double joystickInputY, double joystickInputX, double steerWheelInput) {
        // System.out.println(joystickInputY + " | " + joystickInputX + " | " +
        // steerWheelInput);

        // double acc = Navx.getInstance().getRawAccelX();
        // if (acc > maxAcc) {
        // maxAcc = acc;
        // System.out.println("Max acc: " + acc);
        // }
        // double vel = Navx.getInstance().getVelocityX();
        // if (vel > maxVel) {
        // maxVel = vel;
        // System.out.println("Max vel: " + vel);
        // }
        // MaxVel: 3.35 m/s
        // MaxAcc: 1.20 m/s^6

        // joystickInputY *= 180;

        // double yInput = Math.abs(joystickInputY) < 0.1 && Math.abs(steerInput) > 0.8?
        // 0.1: joystickInputY;

        // System.out.println(joystickInputY);
        // System.out.println(steerInput);

        if (brakeSolenoid.get() == Value.kForward) {
            tankDrive.stopMotor();
            return;
        }

        // Deadzone
        double yInput = deadZone(joystickInputY, 0.02);
        double xInput = deadZone(joystickInputX, 0.05);
        double steerInput = deadZone(steerWheelInput, 0.01);

        // Add direction and sensibility
        steerInput *= steerDirection * steeringWheelSensibility;
        xInput *= steerDirection * joystickSteeringSensibility;
        yInput *= driveDirection * speed;

        // If driving backwards in bidirectional mode, invert the steer direction
        if (yInput < 0 && steerMode == SteerMode.CARLIKE)
            steerInput = -steerInput;

        if (steerWithJoystick) {
            this.driveRaw(yInput, speed * xInput);
        } else {
            Pair<Double, Double> pair = joystickToChassisSpeed(yInput, steerInput);
            this.driveRaw(pair.getFirst(), pair.getSecond());
        }
    }

    public void driveRaw(double speed, double rotation) {
        // if (speed > 0.2)
        // System.out.println("speed:" + speed);
        // if (rotation > 0.2)
        // System.out.println("steer:" + rotation);
        motorspeed = speed;
        tankDrive.arcadeDrive(-rotation, speed, false);
    }

    @Override
    public void setMotorPercent(double right, double left) {
        motors.left().set(-left);
        motors.right().set(right);
    }

    private Pair<Double, Double> joystickToChassisSpeed(double accelerationInput, double steerInput) {

        double velocity = driveFilter.calculate(accelerationInput);
        double steer = steerInput * Math.signum(accelerationInput) * 2;

        // Getting the sign of velocity and steer
        double velocitySign = Math.signum(velocity);
        double steerSign = Math.signum(steer);

        // Squaring the velocity, keeping it's sign
        velocity = Math.abs(velocity) * velocitySign;

        // Calculating the mapped steer and velocity values
        double mappedSteer = Math.min(Math.abs(steer), 1) * velocity * steerSign;
        double mappedVelocity = Math.min(
                Math.abs(Math.abs(velocity) + Math.abs(velocity) * Math.min(0, 1 - Math.abs(steer))),
                Math.abs(velocity)) * velocitySign;

        // Return with those values
        return new Pair<Double, Double>(mappedVelocity, mappedSteer);
    }

    private double deadZone(double val, double dead) {
        if (Math.abs(val) < dead)
            return 0.0;
        return (val - Math.signum(val) * dead) / (1 - dead);
    }

    @Override
    public void reverseDrivingDirection(boolean reverse) {
        driveDirection = reverse ? -1 : 1;
        steerDirection = reverse ? 1 : -1;
    }

    @Override
    public void setSteerMode(SteerMode mode) {
        steerMode = mode;
    }

    @Override
    public void setDirection(int direction) {
        driveDirection = direction;
    }

    @Override
    public void tankDriveVolts(Double leftSpeed, Double rightSpeed) {
        motors.left().setVoltage(leftSpeed * driveDirection);
        motors.right().setVoltage(-rightSpeed * driveDirection);
        tankDrive.feed();
        voltsLeft = leftSpeed * driveDirection;
        voltsRight = rightSpeed * driveDirection;
    }

    /* Getter Methods */

    @Override
    public Pose2d getPosition() {
        return odometry.getPoseMeters();
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                // getEncoderVelocity returns the encodervelocity in ticks / 100ms
                motors.left().getEncoderVelocity() * 10 * (Constants.Drive.Odometry.encoderToMetersConversion)
                        * driveDirection,
                -motors.right().getEncoderVelocity() * 10 * (Constants.Drive.Odometry.encoderToMetersConversion)
                        * driveDirection);
    }

    double voltsRight;
    double voltsLeft;

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    @Override
    public PIDController getRightVelocityController() {
        return leftVelocityController;
    }

    @Override
    public PIDController getLeftVelocityController() {
        return rightVelocityController;
    }

    @Override
    public DifferentialDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    @Override
    public SimpleMotorFeedforward getMotorFeedforward() {
        return motorFeedForward;
    }

    private void resetSensors() {
        FridoNavx.getInstance().reset();
        odometry.resetPosition(
                new Rotation2d(0), 0, 0, new Pose2d(0, 0, new Rotation2d(0)));
        motors.right().setEncoderPosition(0);
        motors.left().setEncoderPosition(0);
    }

    private void configSimpleMotorFeedforward() {
        motorFeedForward = new SimpleMotorFeedforward(
                Constants.Drive.PathWeaver.ksMeters,
                Constants.Drive.PathWeaver.kvMetersPerSecoond,
                Constants.Drive.PathWeaver.ka);
    }

    private void configConstrains() {
        configVoltageConstrain();
        configKinematicsConstrain();
        configCetripedalAccelerationConstrain();
    }

    private void configVoltageConstrain() {
        voltageConstraint = new DifferentialDriveVoltageConstraint(
                motorFeedForward,
                kinematics,
                10);
    }

    private void configKinematicsConstrain() {
        kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
                kinematics,
                Constants.Drive.PathWeaver.kMaxSpeed);
    }

    private void configCetripedalAccelerationConstrain() {
        centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(
                Constants.Drive.PathWeaver.kMaxCentripetalAcceleration);
    }

    private void configTrajectoryConfig() {
        trajectoryConfig = new TrajectoryConfig(
                Constants.Drive.PathWeaver.kMaxSpeed,
                Constants.Drive.PathWeaver.kMaxAcceleration).setKinematics(kinematics).addConstraint(voltageConstraint)
                .addConstraint(kinematicsConstraint).addConstraint(centripetalAccelerationConstraint);
    }

    @Override
    public void stopMotors() {
        tankDrive.stopMotor();
    }

    @Override
    public void stopAndBreak() {
        stopMotors();
        triggerBrake();
        // TODO
    }

    @Override
    public void periodic() {

    }

    public double getRightEncoderDistance() {
        return motors.right().getEncoderTicks() * Constants.Drive.Odometry.encoderToMetersConversion;
    }

    public double getLeftEncoderDistance() {
        return motors.left().getEncoderTicks() * Constants.Drive.Odometry.encoderToMetersConversion;
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public List<Binding> getMappings() {
        Binding driveForward = new Binding(
                Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.driveForward,
                Button::toggleOnTrue, new ReverseDrivingDirection(false));

        Binding driveInverted = new Binding(Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.driveBackward,
                Button::toggleOnTrue, new ReverseDrivingDirection(true));

        Binding carMode = new Binding(
                Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.steerModeCarlike,
                Button::toggleOnTrue, new SetSteerMode(SteerMode.CARLIKE));

        Binding bidirectionalMode = new Binding(
                Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.steerModeBidirectional,
                Button::toggleOnTrue, new SetSteerMode(SteerMode.BIDIRECTIONAL));

        Binding activateBrake = new Binding(
                Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.activateBrake,
                Button::toggleOnTrue, new BrakeCommand());

        Binding activateBalancing = new Binding(
                Constants.Joysticks.accelerator,
                Constants.Drive.ButtonIds.activateBalancing,
                Button::toggleOnTrue, new PIDBalanceCommand());

        Binding slow = new Binding(Constants.Joysticks.accelerator, LogitechExtreme._1, Button::whileTrue,
                new Command() {
                    public void initialize() {
                        speed = slowSpeed;
                    }

                    public void end(boolean interrupted) {
                        speed = defaultDriveSpeed;
                    }

                    public void execute() {
                    }

                    public boolean isFinished() {
                        return false;
                    }
                });

        Binding fast = new Binding(Constants.Joysticks.accelerator, LogitechExtreme._2, Button::whileTrue,
                new Command() {
                    public void initialize() {
                        speed = fastSpeed;
                    }

                    public void end(boolean interrupted) {
                        speed = defaultDriveSpeed;
                    }

                    public void execute() {
                    }

                    public boolean isFinished() {
                        return false;
                    }
                });

        return List.of(
                driveForward, driveInverted,
                carMode, bidirectionalMode,
                activateBrake, slow, fast, activateBalancing);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("steer with joystick", () -> steerWithJoystick, (val) -> steerWithJoystick = val);
        builder.addStringProperty("steer mode", () -> steerMode.name(), null);
        builder.addDoubleProperty("drive driection", () -> driveDirection, null);
        builder.addDoubleProperty("steering wheel sensibility", () -> steeringWheelSensibility,
                (val) -> steeringWheelSensibility = val);
        builder.addDoubleProperty("joystick x (steering) sensibility", () -> joystickSteeringSensibility,
                (val) -> joystickSteeringSensibility = val);
        builder.addDoubleProperty("speed", () -> speed, (val) -> speed = val);
        builder.addDoubleProperty("motorspeed", () -> motorspeed, (val) -> motorspeed = val);
        builder.addDoubleProperty("wheel speed right", () -> this.getWheelSpeeds().rightMetersPerSecond, null);
        builder.addDoubleProperty("wheel speed left", () -> this.getWheelSpeeds().leftMetersPerSecond, null);

        builder.addDoubleProperty("wheel speed diff right - left [m per s]", () -> this.getWheelSpeeds().rightMetersPerSecond - this.getWheelSpeeds().leftMetersPerSecond, null);
        builder.addDoubleProperty("wheel speed left", () -> this.getWheelSpeeds().leftMetersPerSecond, null);
        builder.addDoubleProperty("position_x", () -> this.getPosition().getX(), null);
        builder.addDoubleProperty("position_y", () -> this.getPosition().getY(), null);
        builder.addDoubleProperty("volts right", () -> voltsRight, null);
        builder.addDoubleProperty("volts left", () -> voltsLeft, null);
        builder.addDoubleProperty("Output", () -> motors.right().get(), null);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        builder.addDoubleProperty("speed multiplier", () -> speed, null);
    }

    @Override
    public void triggerBrake() {
        tankDrive.stopMotor();
        brakeSolenoid.set(Value.kForward);
    }

    @Override
    public void releaseBrake() {
        brakeSolenoid.set(Value.kReverse);
    }

    @Override
    public double getPitch() {
        return (double) FridoNavx.getInstance().getRoll();
    }

    @Override
    public void resetOdometry(Pose2d initialPose) {
        FridoNavx.getInstance().reset();
        FridoNavx.getInstance().setAngleAdjustment(initialPose.getRotation().getRadians());
        System.out.println("Navx rot: " + FridoNavx.getInstance().getRotation2d() + ", initialPose rot: " + initialPose.getRotation());
        odometry.resetPosition(initialPose.getRotation(), getLeftEncoderDistance(), getRightEncoderDistance(), initialPose);
    }

    int testbalance = 0;

    public void balance() {
        float pitch = FridoNavx.getInstance().getPitch();
        // System.out.println(pitch);
        balancevalues.add(pitch);
        if (balancevalues.size() == 3) {
            balancevalues.remove(0);
            System.out.println(balancevalues.get(1) - balancevalues.get(0));
            if (balancevalues.get(1) - balancevalues.get(0) < -1) {
                // Drive.getInstance().driveRaw(0,0);
                // Drive.getInstance().triggerBrake();
                testbalance = 1;
                System.out.println("balanced");
            } else {
                // Drive.getInstance().drive(0.4, 0,0);
            }
        }
        if (testbalance == 0) {
            // Drive.getInstance().driveRaw(0.5, 0);
        }
    }
}
