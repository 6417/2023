package frc.robot.commands.balance;

import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.TimedForward;
import frc.robot.subsystems.drive.Drive;

public class PIDBalanceCommand extends SequentialCommandGroup {
    private static final double BASE_VELOCITY = 0.6;

    public PIDBalanceCommand() {
        super(
            new FindChargingStation(BASE_VELOCITY),
            new TimedForward(1, BASE_VELOCITY),
            new DriveUntilEven(),
            new KeepBalance()
        );
        requires(Drive.getInstance());
    }
}
