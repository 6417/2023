package frc.robot.commands.autonomous;

import frc.fridowpi.command.ParallelRaceGroup;
import frc.robot.subsystems.drive.Drive;

public class DriveOnChargingStation extends ParallelRaceGroup {

    public DriveOnChargingStation() {
        Drive.getInstance();
    }
}
