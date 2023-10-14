package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.OpenCloseClaw;
import frc.robot.commands.claw.SetClawBeltSpeed;
import frc.robot.commands.claw.SpitPiece;
import frc.robot.constants.ArmShoulderPositions;

public class ScorePreloadedPiece extends SequentialCommandGroup{
    public ScorePreloadedPiece () {
        addCommands(
            // Grip cone.
            new OpenCloseClaw(false),
            new SetClawBeltSpeed(0.2),

            // Raise arm, drop cone, and stow.
            new SetArmShoulderPosition(ArmShoulderPositions.Cone.MID),
            new SpitPiece()
        );
    }
}
