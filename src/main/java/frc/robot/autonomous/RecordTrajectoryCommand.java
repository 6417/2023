package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class RecordTrajectoryCommand extends CommandBase{
    TrajectoryCreator logger;
    Timer timer;
    double cooldown = Drive.Constants.Autonomous.recordingCooldownSeconds;
    double previousTIme;

    public RecordTrajectoryCommand() {
    }

    @Override
    public void initialize() {
        logger = new TrajectoryCreator("tmp/TrajectoryRecording.wpilib.json");
        Drive.getInstance().resetSensors();
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        cooldown -= (timer.get() - previousTIme);
        previousTIme = timer.get();
        if (cooldown <= 0) {
            logger.addDatapoint(Drive.getInstance().getChassisSpeeds(), Drive.getInstance().getPosition());
            cooldown = Drive.Constants.Autonomous.recordingCooldownSeconds;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(Drive.getInstance().getPosition().toString());
        logger.close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}