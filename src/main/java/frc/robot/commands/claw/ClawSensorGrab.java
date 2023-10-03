package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Claw;


public class ClawSensorGrab extends SequentialCommandGroup {
  private final Claw m_claw;

  public ClawSensorGrab() {
    m_claw = Claw.getInstance();

    addRequirements(m_claw);
    addCommands(
      new OpenCloseClaw(true),
      new SetClawBeltSpeed(0.8),

      new WaitUntilCommand(
        () -> m_claw.getIRSensorTriggered()
      ),

      new OpenCloseClaw(false),
      new SetClawBeltSpeed(0.2)
    );
  }
}
