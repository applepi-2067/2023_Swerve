package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class AutoBalance extends CommandBase {
  private final Drivetrain m_drivetrain;
  
  public AutoBalance() {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: Check sign.
    double leftStickY = Math.sin(-1.0 * m_drivetrain.getGyroPitch()) / 2.0;
    m_drivetrain.drive(0.0, leftStickY, 0.0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Keep balancing, never finish.
    return false;
  }
}
