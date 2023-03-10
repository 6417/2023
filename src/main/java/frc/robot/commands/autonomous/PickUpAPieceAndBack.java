package frc.robot.commands.autonomous;

import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.Utils;
import frc.robot.Utils.PiecePosition;


public class PickUpAPieceAndBack extends SequentialCommandGroup {

  public PickUpAPieceAndBack(Utils pos, PiecePosition piece) {
    String pathToPiece = pos.toString() + "To" + piece.toString();

    addCommands(
        new FollowPath(pathToPiece),
        new GrabPieceFront(),
        new FollowPath(pathToPiece + "Inverted")
    );
  }
}