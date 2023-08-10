package frc.robot.subsystems.swerve.steer;

public interface SteerMotor {
    public void setTargetPositionDegrees(double targetPositionDegrees);
    
    public double getPositionDegrees();
}
