package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;


public class SetShoulderPosition extends CommandBase {
  private final Shoulder m_shoulder;

  private double m_degrees;

  public SetShoulderPosition(double degrees) {
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);

    m_degrees = degrees;
  }

  @Override
  public void initialize() {
    m_shoulder.setTargetPositionDegrees(m_degrees);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double error = Math.abs(m_shoulder.getPositionDegrees() - m_degrees);
    return error < Shoulder.TARGET_ANGLE_TOLERANCE_DEGREES;
  }
}
