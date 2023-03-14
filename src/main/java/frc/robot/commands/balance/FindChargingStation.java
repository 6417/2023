package frc.robot.commands.balance;

// import frc.llib.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

// After this command finishes it doesn't stop the motors
public class FindChargingStation extends CommandBase {
    private double base_velocity;
    public double pitch;
    private double THRESHOLD = 10;

    public FindChargingStation(double velocity) {
        addRequirements(Drive.getInstance());
        this.base_velocity = velocity;
    }

    @Override
    public void initialize() {
        // CommandScheduler.getInstance().cancel(Drive.getInstance().getDefaultCommand());
        // CommandScheduler.getInstance().removeDefaultCommand(Drive.getInstance());
    }

    // (Aka .periodic())
    @Override
    public void execute() {
        Drive.getInstance().driveRaw(base_velocity, 0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("[FindChargingStation] Interrupted");
        } else {
            System.out.println("[FindChargingStation] Found obstacle");
        }
    }

    @Override
    public boolean isFinished() {
        pitch = Drive.getInstance().getPitch();
        return Math.abs(pitch) > THRESHOLD;
    }
}
