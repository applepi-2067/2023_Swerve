package frc.robot.subsystems.swerve.steer;

public interface SteerMotor {
    public void setTargetPositionDegrees(double targetPositionDegrees);

    public double getRelativePositionTicks();
    public double getAbsolutePositionTicks();
    
    public double getPositionDegrees();

    public double getTicksPerRev();
}
