package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Claw;


public class ClawSensorGrab extends SequentialCommandGroup {
  public ClawSensorGrab() {
    addCommands(
      new OpenCloseClaw(true),
      new SetClawBeltSpeed(0.8),

      new WaitUntilCommand(
        () -> Claw.getInstance().getIRSensorTriggered()
      ),

      new OpenCloseClaw(false),
      new SetClawBeltSpeed(0.4)
    );
  }
}
