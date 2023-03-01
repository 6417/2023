package frc.robot.commands.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.command.Command;
import frc.robot.autonomous_tools.PathviewerLoader;
import frc.robot.autonomous_tools.RamseteCommandGenerator;
import frc.robot.subsystems.drive.Drive;


public class FollowPath extends Command {
    edu.wpi.first.wpilibj2.command.Command autonomousCommand;
    private Trajectory path;

    public FollowPath(String name) {
        path = PathviewerLoader.loadTrajectory("paths/" + name + ".wpilib.json");
    }

    @Override
    public void initialize() {
        autonomousCommand = RamseteCommandGenerator.generateRamseteCommand(this.path);
        CommandScheduler.getInstance().schedule(autonomousCommand);

        Drive.getInstance().setDirection(1);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(autonomousCommand);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished");
        // Drive.getInstance().setDirection(1);
    }
}
