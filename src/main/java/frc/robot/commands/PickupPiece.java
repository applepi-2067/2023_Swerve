package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.constants.ArmShoulderPositions;
import frc.robot.utils.ArmShoulderPosition;


public class PickupPiece extends SequentialCommandGroup {
  public PickupPiece(ArmShoulderPosition armShoulderPosition) {
    addCommands(
      new SetArmShoulderPosition(armShoulderPosition),
      new ClawSensorGrab(),
      new SetArmShoulderPosition(ArmShoulderPositions.STOW)
    );
  }
}
