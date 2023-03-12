package frc.robot;

import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.drive.Drive;

public class Utils {
    public static DriveBase drive() {
        return Drive.getInstance();
    }
}
