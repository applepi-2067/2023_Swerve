package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.swerve.drive.*;
import frc.robot.subsystems.swerve.steer.*;

public class SwerveModule {
    private DriveMotor m_driveMotor;
    private SteerMotor m_steerMotor;

    public SwerveModule(int location) {
        m_driveMotor = new SparkMaxDriveMotor(location);
        m_steerMotor = new TalonSRXSteerMotor(location);
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Optimize state.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            targetState, Rotation2d.fromDegrees(m_steerMotor.getPositionDegrees())
        );

        // Set steer motor to target rotation.
        double targetDegrees = optimizedState.angle.getDegrees();
        m_steerMotor.setTargetPositionDegrees(targetDegrees);

        // Set drive motor to target speed.
        double targetSpeedMetersPerSecond = optimizedState.speedMetersPerSecond;
        m_driveMotor.setTargetVelocityMetersPerSecond(targetSpeedMetersPerSecond);
    }
}
