package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SetArmShoulderPosition;
import frc.robot.constants.ScoringPositions;
import frc.robot.subsystems.Claw;


public class SpitPiece extends SequentialCommandGroup {
  public SpitPiece() {
    addCommands(
      new OpenCloseClaw(true),
      new SetClawBeltSpeed(-0.5),

      new WaitUntilCommand(
        () -> !Claw.getInstance().getIRSensorTriggered()
      ),

      new SetClawBeltSpeed(0.0),
      new OpenCloseClaw(false),

      new SetArmShoulderPosition(ScoringPositions.STOW)
    );
  }
}
