package frc.robot.commands.autonomous;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedForward extends WaitCommand {
    private double vel;

    public TimedForward(double secs, double vel) {
        super(secs);
        addRequirements(Drive.getInstance());
        this.vel = vel;
    }

    // Called every time the scheduler runs while the command is scheduled.
    // (Aka .periodic())
    @Override
    public void execute() {
        Drive.getInstance().driveRaw(vel, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Drive.getInstance().stopMotors();
    }
}
