package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.swerve.drive.*;
import frc.robot.subsystems.swerve.steer.*;

public class SwerveModule {
    private DriveMotor m_driveMotor;
    private SteerMotor m_steerMotor;

    private int location;

    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
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

    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
            m_driveMotor.getVelocityMetersPerSecond(), Rotation2d.fromDegrees(m_steerMotor.getPositionDegrees())
        );
        return state;
    }

    public String getDescription() {
        SwerveModuleState state = getState();

        String description = "Loc " + location + ": ";
        description += "angle (deg)=" + state.angle.getDegrees() + "    ";
        description += "angle (rel ticks)=" + m_steerMotor.getRelativePositionTicks() + "    ";
        description += "angle (abs ticks)=" + m_steerMotor.getAbsolutePositionTicks() + "    ";
        description += "v (m/s)=" + state.speedMetersPerSecond + "    ";
        return description;
    }
}
