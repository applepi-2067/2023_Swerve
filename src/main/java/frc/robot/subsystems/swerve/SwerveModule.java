package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.drive.*;
import frc.robot.subsystems.swerve.steer.*;

public class SwerveModule {
    // TODO: find steer wheel zero offsets.
    
    private static final class CAN_IDS {
        // Follows back left to front right convention.
        private static final int[] DRIVE = {1, 2, 3, 4};
        private static final int[] STEER = {5, 6, 7, 8};
    }

    private static final double[] STEER_WHEEL_ZERO_OFFSET_TICKS = {0.0, 0.0, 0.0, 0.0};

    private int location;

    private DriveMotor m_driveMotor;
    private SteerMotor m_steerMotor;

    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
        m_driveMotor = new TalonFXDriveMotor(CAN_IDS.DRIVE[location]);
        m_steerMotor = new SparkMaxSteerMotor(
            CAN_IDS.STEER[location],
            STEER_WHEEL_ZERO_OFFSET_TICKS[location]
        );
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
        description += "v (m/s)=" + state.speedMetersPerSecond + "    ";
        return description;
    }
}
