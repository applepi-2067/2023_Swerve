package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.constants.ArmShoulderPositions;
import frc.robot.utils.ArmShoulderPosition;


public class PickupPiece extends SequentialCommandGroup {
  public PickupPiece(ArmShoulderPosition armShoulderPosition) {
    this(armShoulderPosition, false);
  }

  public PickupPiece(ArmShoulderPosition armShoulderPosition, boolean raiseBeforeStow) {
    addCommands(
      new SetArmShoulderPosition(armShoulderPosition),
      new ClawSensorGrab()
    );

    if (raiseBeforeStow) {
      ArmShoulderPosition raisedArmShoulderPosition = new ArmShoulderPosition(
        armShoulderPosition.m_armPositionMeters,
        armShoulderPosition.m_shoulderAngleDegrees + 10.0
      );

      addCommands(new SetArmShoulderPosition(raisedArmShoulderPosition, false));
    }

    addCommands(new SetArmShoulderPosition(ArmShoulderPositions.STOW));
  }
}