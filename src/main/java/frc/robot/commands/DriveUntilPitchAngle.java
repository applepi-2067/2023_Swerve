package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class DriveUntilPitchAngle extends CommandBase {
  private final Drivetrain m_drivetrain;

  private final double m_targetPitchDegrees;
  
  public DriveUntilPitchAngle(double targetPitchDegrees) {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    m_targetPitchDegrees = targetPitchDegrees;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrain.drive(0.0, 0.5, 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getGyroPitch()) >= m_targetPitchDegrees;
  }
}
