package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ZeroArm extends CommandBase {
  private Arm m_arm;

  public ZeroArm() {
    m_arm = Arm.getInstance();
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: Robot setup instructions.
    m_arm.setPercentOutput(-1.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setEncoderPositionMeters(Arm.MAGNET_SENSOR_POSITION_METERS);
    m_arm.setTargetPositionMeters(0.0);
  }

  @Override
  public boolean isFinished() {
    return m_arm.getZeroSensorTriggered();
  }
}
