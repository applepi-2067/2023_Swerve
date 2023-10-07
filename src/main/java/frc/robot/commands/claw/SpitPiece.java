package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class SpitPiece extends SequentialCommandGroup {
  public SpitPiece() {
    addCommands(
      new OpenCloseClaw(true),
      new SetClawBeltSpeed(-1.0)
    );
  }
}
