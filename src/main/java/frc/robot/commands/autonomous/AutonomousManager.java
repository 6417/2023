package frc.robot.commands.autonomous;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.command.Command;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonomous.armControlCommands.GrabPieceFront;
import frc.robot.commands.autonomous.armControlCommands.PlacePieceBack;
import frc.robot.commands.balance.PIDBalanceCommand;
import frc.robot.subsystems.drive.Drive;

// Assembles the different autonomous commands to complete autonomous commands
public class AutonomousManager {

    // Follow a trajectory described through predefined waypoints
    public static Command getPathCommand(List<PreDefPose> waypoints) {
        assert waypoints != null : "[AutonomousErr] waypoints is null";
        assert waypoints.size() > 2 : "[AutonomousErr] get less then two waypoints";

        // Extract path name by adding the single waypoints as strings together
        String pathName = new String();
        for (var p : waypoints) {
            pathName += p.toString() + "_";
        }
        // Strip trailing underscore
        pathName = pathName.substring(0, pathName.length() - 1);

        // Reset odometry to starting thi first waypoint
        Pose2d startingPose = waypoints.get(0).toPose();
        System.out.println("[Autonomous] Setting odometry to " + startingPose);
        Drive.getInstance().resetOdometry(startingPose);

        // Load path
        System.out.println("[Autonomous] Loading path" + pathName);
        return new FollowPath(pathName);
    }

    public static Command generatePathCommand(List<Pose2d> waypoints) {
        assert waypoints != null : "[AutonomousErr] waypoints is null";
        assert waypoints.size() > 2 : "[AutonomousErr] get less then two waypoints";

        Pose2d startingPose = waypoints.get(0);
        System.out.println("[Autonomous] Setting odometry to " + startingPose);
        Drive.getInstance().resetOdometry(startingPose);


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, new TrajectoryConfig(
                Constants.Drive.PathWeaver.kvMetersPerSecoond, Constants.Drive.PathWeaver.kMaxAcceleration));


        return new FollowPath(trajectory);
    }

    // Simply engage with the charging station
    public static CommandBase getCommand_DriveOnCharginStation() {
        return new PIDBalanceCommand();
    }

    public static CommandBase getCommand_GrabPieceSimpleAndEngage(Alliance alliance, PreDefPose start,
            PreDefPose piece) {
        return new SequentialCommandGroup(
                new PlacePieceBack(),
                new FollowPath(start.toString() + "_" + piece.toString()),
                new GrabPieceFront(),
                new FollowPath(piece.toString() + "_" + alliance.toString() + "ChStIn"),
                new PIDBalanceCommand());
    }

    // Starting position: BlueLineRight or RedLineLeft
    // Waypoints : OVER charging station to a (middle) piece
    // Finishes : On the charging station
    public static CommandBase getCommand_GrabPiece_OVER_ChStAndEngage(Alliance alliance, PreDefPose piece) {
        return new SequentialCommandGroup(
                new PlacePieceBack(),
                new FollowPath(alliance + "Line" + (alliance == Alliance.Blue ? "Right" : "Left") + "_ChSt_" + piece),
                new FollowPath(alliance + "Piece" + piece + "_" + alliance + "ChStIn"),
                new PIDBalanceCommand());
    }

    // Starting position: BlueLineRight or RedLineLeft
    // Waypoints : AROUND charging station (closest way) to a piece
    // Finishes : (Hopefully) engaged with the charging station
    public static CommandBase getCommand_GrapPiece_AROUND_ChStAndEngage(Alliance alliance) {
        return new SequentialCommandGroup(
                new PlacePieceBack(),
                new FollowPath(getPath_LineToPiece(alliance)),
                new GrabPieceFront(),
                new FollowPath(getPath_PieceToCharginStation(alliance)));
    }

    // Helper functions

    private static String getPath_LineToPiece(Alliance alliance) {
        return alliance == Alliance.Blue ? "BlueLineRight_BluePiece2" : "RedLineLeft_RedPiece3";
    }

    private static String getPath_PieceToCharginStation(Alliance alliance) {
        return alliance == Alliance.Blue ? "BluePiece2_BlueChStIn" : "RedPiece3_RedChStIn";
    }
}
