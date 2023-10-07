package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;


public class SetArmShoulderPositions extends SequentialCommandGroup {
  public SetArmShoulderPositions(double meters, double degrees) {
    addCommands(
      new SetArmPosition(0.0),
      new SetShoulderPosition(degrees),
      new SetArmPosition(meters)
    );
  }
}
