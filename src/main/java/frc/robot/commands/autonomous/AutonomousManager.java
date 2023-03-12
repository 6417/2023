package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.command.Command;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.PreDefPose.PoseOnField;
import frc.robot.commands.autonomous.armControlCommands.GrabPieceFront;
import frc.robot.commands.autonomous.armControlCommands.PlacePieceBack;
import frc.robot.commands.balance.PIDBalanceCommand;
import frc.robot.subsystems.drive.Drive;

public class AutonomousManager {

    public static Command getPathCommand(List<PoseOnField> waypoints) {
        assert waypoints != null : "[AutonomousErr] waypoints is null";
        assert waypoints.size() > 2 : "[AutonomousErr] get less then two waypoints";

        String pathName = new String();
        for (var p : waypoints) {
            pathName += new PreDefPose(p).toString() + "_";
        }
        // Strip trailing underscore
        pathName = pathName.substring(0, pathName.length() - 1);

        Pose2d startingPose = new PreDefPose(waypoints.get(0)).toPose();

        System.out.println("[Autonomous] Setting odometry to " + startingPose);
        Drive.getInstance().resetOdometry(startingPose);

        System.out.println("[Autonomous] Loading path" + pathName);
        return new FollowPath(pathName);
    }

    public static CommandBase getCommand_DriveOnCharginStation() {
        return new PIDBalanceCommand();
    }

    public static CommandBase getCommand_Line_ChSt_Piece_ChSt(String allianceColor, int piece) {
        return new SequentialCommandGroup(
                new PlacePieceBack(),
                new FollowPath(allianceColor + "Line" + allianceColor == "Blue"? "Right": "Left" + "_ChSt_Piece" + piece),
                new FollowPath(allianceColor + "Piece" + piece + "_" + allianceColor + "ChStIn"),
                new PIDBalanceCommand());
    }

    public static CommandBase getCommand_GrapPieceAndChargingStation(Alliance alliance) {
        return new SequentialCommandGroup(
                new PlacePieceBack(),
                new FollowPath(getPath_LineToPiece(alliance)),
                new GrabPieceFront(),
                new FollowPath(getPath_PieceToCharginStation(alliance)));
    }

    private static String getPath_LineToPiece(Alliance alliance) {
        return alliance == Alliance.Blue ? "BlueLineRight_BluePiece2" : "RedLineLeft_RedPiece3";
    }

    private static String getPath_PieceToCharginStation(Alliance alliance) {
        return alliance == Alliance.Blue ? "BluePiece2_BlueChStIn" : "RedPiece3_RedChStIn";
    }
}
