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
    m_shoulder.setEncoderPosition(0.0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
