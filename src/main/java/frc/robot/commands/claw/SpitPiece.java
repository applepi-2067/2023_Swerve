package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SetArmShoulderPosition;
import frc.robot.constants.ArmShoulderPositions;
import frc.robot.subsystems.Claw;


public class SpitPiece extends SequentialCommandGroup {
  public SpitPiece() {
    addCommands(
      new OpenCloseClaw(true),
      new SetClawBeltSpeed(-0.7),

      new WaitUntilCommand(
        () -> !Claw.getInstance().getIRSensorTriggered()
      ),
      new WaitCommand(0.2),

      new SetClawBeltSpeed(0.0),
      new OpenCloseClaw(false),

      new SetArmShoulderPosition(ArmShoulderPositions.STOW)
    );
  }
}
