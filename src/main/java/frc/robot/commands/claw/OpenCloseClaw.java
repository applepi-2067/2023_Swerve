package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;


public class OpenCloseClaw extends InstantCommand {
  private final Claw m_claw;

  private boolean m_open;

  public OpenCloseClaw(boolean open) {
    m_claw = Claw.getInstance();
    addRequirements(m_claw);

    m_open = open;
  }

  @Override
  public void initialize() {
    if (m_open) {
      m_claw.openClaw();
    }
    else {
      m_claw.closeClaw();
    }
  }
}
