package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;

public class Utils {
    public static DriveBase drive() {
        return Drive.getInstance();
    }
}