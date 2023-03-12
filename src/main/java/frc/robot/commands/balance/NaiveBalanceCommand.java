package frc.robot.commands.balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.drive.Drive;

public class NaiveBalanceCommand extends CommandBase {
    public DriveBase subsystem;
    private final double PITCH_WEIGHT = 1 / 5;
    private final double BASE_VELOCITY = 0.35;
    private final double PITCH_DEAD_BAND = 3;
    private double vel = 0;

    private double pitch;
    private double prev_pitch;
    private BState state;
    private BState prev_state;
    private PIDController pid;

    public static enum BState {
        Searching, DrivingUp, KeepingBalance
    }

    static boolean firstInstance = true;

    public NaiveBalanceCommand() {
        pid = new PIDController(0.001, 0, 0);
        subsystem = Drive.getInstance();
        addRequirements(subsystem);
        if (firstInstance) {
            Shuffleboard.getTab("Debug").add("pitch", new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                    builder.addDoubleProperty("Pitch", () -> pitch, null);
                }
            });
            firstInstance = false;
        }
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
            System.out.println("NEW STATE: " + state);
        prev_state = state;

        switch (state) {
            case Searching:
                vel = BASE_VELOCITY;
                if (Math.abs(pitch) >= 13) {
                    System.out.println("found target");
                    state = BState.DrivingUp;
                    pid.reset();
                    pid.calculate(pitch, 0);
                }
                break;
            case DrivingUp:

                vel = pid.calculate(pitch);
                // System.out.println(vel);
                // if (pitch < PITCH_DEAD_BAND * 2.5) {
                // System.out.println("Increasing speeed");
                // vel *= 1.1;
                // }
                // if (prev_pitch - pitch > 7) {
                //     vel = 0;
                //     System.out.println("breaking");
                //     subsystem.stopAndBreak();
                // }
                // if ((Math.abs(prev_pitch) + Math.abs(pitch)) < 1) {
                    // state = BState.KeepingBalance;
                // }
                break;
            case KeepingBalance:
                vel = 0;
                if (Math.abs(pitch) > PITCH_DEAD_BAND) {
                    System.out.println("Instable! \nReactivating Balancing Code");
                    state = BState.DrivingUp;
                }
                break;
        }
        subsystem.driveRaw(vel, 0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("BalancingCommand finished");
        subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
