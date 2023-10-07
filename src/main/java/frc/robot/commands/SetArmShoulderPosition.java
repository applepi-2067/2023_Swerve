package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.utils.ScoringPosition;


public class SetArmShoulderPosition extends SequentialCommandGroup {
  public SetArmShoulderPosition(ScoringPosition scoringPosition) {
    addCommands(
      new SetArmPosition(0.0),
      new SetShoulderPosition(scoringPosition.m_shoulderAngleDegrees),
      new SetArmPosition(scoringPosition.m_armPositionMeters)
    );
  }
}
