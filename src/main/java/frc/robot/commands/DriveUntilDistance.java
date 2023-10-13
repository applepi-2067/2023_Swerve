package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class DriveUntilDistance extends CommandBase {
  private final Drivetrain m_drivetrain;

  private final double m_distanceMeters;
  
  private double m_initialRobotPoseX;
  
  public DriveUntilDistance(double distanceMeters) {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    m_distanceMeters = distanceMeters;

    m_initialRobotPoseX = m_drivetrain.getRobotPose2d().getX();
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
    return Math.abs(m_drivetrain.getRobotPose2d().getX() - m_initialRobotPoseX) >= m_distanceMeters;
  }
}
