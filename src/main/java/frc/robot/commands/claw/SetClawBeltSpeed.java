package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;


public class SetClawBeltSpeed extends InstantCommand {
  private final Claw m_claw;

  private double m_percentOutput;

  public SetClawBeltSpeed(double percentOutput) {
    m_claw = Claw.getInstance();
    addRequirements(m_claw);

    m_percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    m_claw.setPercentOutput(m_percentOutput);
  }
}
