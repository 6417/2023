
package frc.robot.commands.autonomous;

import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.StartingPosition;


public class ChargeAutonomous extends SequentialCommandGroup {

  public ChargeAutonomous(StartingPosition startingPosition) {
    addCommands(
    //   new TimedForward(1),
      new FollowPath("forward_speed2")
    //   new FollowPath("StraitRight")
    );
    // switch (startingPosition) {
    //   case LEFT:
    //     addCommands(
    //       new FollowPath("LeftToChSt")
    //       // new TimedForward(1)
    //     );
    //     break;
    //   case MID: break;
    //   case RIGHT: break;
    // }
  }
}