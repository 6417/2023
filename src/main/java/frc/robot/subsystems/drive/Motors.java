package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.fridowpi.motors.FridoCanSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class Motors {
    // Class for controlling and grouping all motors in the chassis
    // Note: works with 4 Falcon motors

    private FridoCanSparkMax masterRight;
    private FridoCanSparkMax masterLeft;
    private FridoCanSparkMax followerRight;
    private FridoCanSparkMax followerLeft;

    public Motors() {
        masterRight = new FridoCanSparkMax(Constants.Drive.Motors.FRONTRIGHT, MotorType.kBrushless);
        masterLeft = new FridoCanSparkMax(Constants.Drive.Motors.FRONTRIGHT, MotorType.kBrushless);
        followerRight = new FridoCanSparkMax(Constants.Drive.Motors.BACKRIGTH, MotorType.kBrushless);
        followerLeft = new FridoCanSparkMax(Constants.Drive.Motors.BACKLEFT, MotorType.kBrushless);
    }

    public void init() {
        masterRight.factoryDefault();
        masterLeft.factoryDefault();
        followerRight.factoryDefault();
        followerLeft.factoryDefault();

        masterRight.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        masterLeft.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        followerRight.setIdleMode(FridolinsMotor.IdleMode.kBrake);
        followerLeft.setIdleMode(FridolinsMotor.IdleMode.kBrake);

        followerRight.follow(masterRight);
        followerLeft.follow(masterLeft);

        System.out.println("Motors init completed");
    }

    public FridoCanSparkMax right() {
        return masterRight;
    }

    public FridoCanSparkMax left() {
        return masterLeft;
    }
}
