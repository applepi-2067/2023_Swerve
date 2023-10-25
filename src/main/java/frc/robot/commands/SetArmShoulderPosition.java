package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.utils.ArmShoulderPosition;


public class SetArmShoulderPosition extends SequentialCommandGroup {
  public SetArmShoulderPosition(ArmShoulderPosition armShoulderPosition) {
    this(armShoulderPosition, true);
  }

  public SetArmShoulderPosition(ArmShoulderPosition armShoulderPosition, boolean stowFirst) {
    if (stowFirst) {
      addCommands(
        new SetArmPosition(0.0)
      );
    }
    addCommands(
      new SetShoulderPosition(armShoulderPosition.m_shoulderAngleDegrees),
      new SetArmPosition(armShoulderPosition.m_armPositionMeters),
    );
  }
}