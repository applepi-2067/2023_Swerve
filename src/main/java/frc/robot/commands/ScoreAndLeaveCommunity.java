package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoPositions;


public class ScoreAndLeaveCommunity extends SequentialCommandGroup {
  public ScoreAndLeaveCommunity() {
    addCommands(
      new ScorePreloadedPiece(),
      new DriveUntilDistance(AutoPositions.LEAVE_COMMUNITY_METERS)
    );
  }
}
