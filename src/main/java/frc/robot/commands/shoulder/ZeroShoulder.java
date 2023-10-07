package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;


public class ZeroShoulder extends CommandBase {
  private Shoulder m_shoulder;

  public ZeroShoulder() {
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: Robot setup instructions.
    m_shoulder.setPercentOutput(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_shoulder.setEncoderPositionDegrees(Shoulder.MAGNET_SENSOR_ANGLE_DEGREES);
    m_shoulder.setTargetPositionDegrees(0.0);
  }

  @Override
  public boolean isFinished() {
    return m_shoulder.getZeroSensorTriggered();
  }
}
