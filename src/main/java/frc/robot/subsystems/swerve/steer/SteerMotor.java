package frc.robot.subsystems.swerve.steer;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SteerMotor {
    public void setTargetPositionRotation2d(Rotation2d targetPositionRotation2d);
    
    public Rotation2d getPositionRotation2d();
}
