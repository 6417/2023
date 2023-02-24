// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.StartingPosition;


public class ChargeAutonomous extends SequentialCommandGroup {

  public ChargeAutonomous(StartingPosition startingPosition) {
    addCommands(
      new TimedForward(1),
      new FollowPath("LeftToChSt")
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
