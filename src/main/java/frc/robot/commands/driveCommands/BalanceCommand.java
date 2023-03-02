package frc.robot.commands.driveCommands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;

import org.apache.logging.log4j.core.layout.SyslogLayout;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {
    public DriveBase subsystem;
    private final double PITCH_WEIGHT = 1 / 5;
    private final double BASE_VELOCITY = 0.35;
    private final double PITCH_DEAD_BAND = 3;
    private double vel = 0;

    private double pitch;
    private double prev_pitch;
    private BState state;
    private BState prev_state;

    public static enum BState {
        Searching, DrivingUp, KeepingBalance
    }

    public BalanceCommand() {
        subsystem = Drive.getInstance();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        state = BState.Searching;
        prev_state = state;
        pitch = subsystem.getPitch();
    }

    @Override
    public void execute() {
        prev_pitch = pitch;
        pitch = subsystem.getPitch();
        if (prev_state != state)
            System.out.println("NEW STATE:                              " + state);
        prev_state = state;

        switch (state) {
            case Searching:
                vel = BASE_VELOCITY;
                if (Math.abs(pitch) > PITCH_DEAD_BAND) {
                    state = BState.DrivingUp;
                }
                break;
            case DrivingUp:
                // vel = BASE_VELOCITY * pitch / 15;
                if (pitch < PITCH_DEAD_BAND * 2.5) {
                    vel *= 1.1;
                }
                if (prev_pitch - pitch > -5)
                    vel = 0;
                    subsystem.stopAndBreak();
                }
                if ((Math.abs(prev_pitch) + Math.abs(pitch)) < 1) {
                    state = BState.KeepingBalance;
                }
                break;
            case KeepingBalance:
                vel = 0;
                if (Math.abs(pitch) > PITCH_DEAD_BAND) {
                    state = BState.DrivingUp;
                }
                break;
        }
        subsystem.drive(-vel, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
