package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class CenterScoreAndBalance extends SequentialCommandGroup {

  public CenterScoreAndBalance() {
    addCommands(
      new ScorePreloadedPiece(),
      new DriveUntilPitchAngle(15.0),
      new AutoBalance()
    );
  }
}
