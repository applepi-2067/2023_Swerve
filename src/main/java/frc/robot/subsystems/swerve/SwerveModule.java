package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private DriveMotor m_driveMotor;
    private SteerMotor m_steerMotor;

    public SwerveModule(int driveMotorCAN_ID, int steerMotorCAN_ID) {
        m_driveMotor = new DriveMotor(driveMotorCAN_ID);
        m_steerMotor = new SteerMotor(steerMotorCAN_ID);
    }

    public void drive(SwerveModuleState state) {
        // Optimize state.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            state, Rotation2d.fromDegrees(m_steerMotor.getPositionDegrees())
        );

        // Set steer motor to target rotation.
        double targetDegrees = optimizedState.angle.getDegrees();
        m_steerMotor.setPositionDegrees(targetDegrees);

        // Set drive motor to target speed.
        double targetSpeedMetersPerSecond = optimizedState.speedMetersPerSecond;
        m_driveMotor.setVelocity(targetSpeedMetersPerSecond);
    }
}
