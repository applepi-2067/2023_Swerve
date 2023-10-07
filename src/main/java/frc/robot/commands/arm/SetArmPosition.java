package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class SetArmPosition extends CommandBase {
  private final Arm m_arm;

  private double m_meters;

  public SetArmPosition(double meters) {
    m_arm = Arm.getInstance();
    addRequirements(m_arm);

    m_meters = meters;
  }

  @Override
  public void initialize() {
    m_arm.setTargetPositionMeters(m_meters);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double error = Math.abs(m_arm.getPositionMeters() - m_meters);
    return error < Arm.TARGET_POSITION_TOLERANCE_METERS;
  }
}
