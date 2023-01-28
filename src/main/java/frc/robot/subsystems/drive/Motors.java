package frc.robot.subsystems.drive;

import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class Motors {
    // Class for controlling and grouping all motors in the chassis
    // Note: works with 4 Falcon motors

    private FridoFalcon500 masterRight;
    private FridoFalcon500 masterLeft;
    private FridoFalcon500 followerRight;
    private FridoFalcon500 followerLeft;

    public Motors() {
        masterRight = new FridoFalcon500(Constants.Drive.Motors.FRONTRIGHT);
        masterLeft = new FridoFalcon500(Constants.Drive.Motors.FRONTRIGHT);
        followerRight = new FridoFalcon500(Constants.Drive.Motors.BACKRIGTH);
        followerLeft = new FridoFalcon500(Constants.Drive.Motors.BACKLEFT);
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

    public FridoFalcon500 right() {
        return masterRight;
    }

    public FridoFalcon500 left() {
        return masterLeft;
    }
}
