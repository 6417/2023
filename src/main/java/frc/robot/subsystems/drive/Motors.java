package frc.robot.subsystems.drive;

import frc.fridowpi.motors.*;
import frc.robot.Constants;

public class Motors {
    // Class for controlling and grouping all motors in the chassis
    // Note: works with 4 Falcon motors, where the ones on the front are the masters

    private FridoFalcon500 masterRight;
    private FridoFalcon500 masterLeft;
    private FridoFalcon500 followerRight;
    private FridoFalcon500 followerLeft;

    public Motors() {
        masterRight = new FridoFalcon500(Constants.Drive.Motors.FRONTRIGHT);
        masterLeft = new FridoFalcon500(Constants.Drive.Motors.FRONTLEFT);
        followerRight = new FridoFalcon500(Constants.Drive.Motors.BACKRIGHT);
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
        
        // masterRight.setIdleMode(FridolinsMotor.IdleMode.kCoast);
        // masterLeft.setIdleMode(FridolinsMotor.IdleMode.kCoast);
        // followerRight.setIdleMode(FridolinsMotor.IdleMode.kCoast);
        // followerLeft.setIdleMode(FridolinsMotor.IdleMode.kCoast);

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
