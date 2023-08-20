package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.drive.*;
import frc.robot.subsystems.swerve.steer.*;

public class SwerveModule {
    
    // Follows back left to front right convention.
    private static final class CAN_IDS {
        private static final int[] DRIVE = {1, 2, 3, 4};
        private static final int[] STEER = {5, 6, 7, 8};
    }

    // Reported abs encoder position at wheel zero.
    private static final double[] STEER_WHEEL_ZERO_OFFSET_DEGREES = {34.7, 344.0, 233.3, 69.0};

    private int location;

    private DriveMotor m_driveMotor;
    private SteerMotor m_steerMotor;

    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
        m_driveMotor = new TalonFXDriveMotor(CAN_IDS.DRIVE[location]);
        m_steerMotor = new SparkMaxSteerMotor(
            CAN_IDS.STEER[location],
            STEER_WHEEL_ZERO_OFFSET_DEGREES[location]
        );
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Optimize state.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            targetState, m_steerMotor.getPositionRotation2d()
        );

        // Set steer and drive motors to targets.
        m_steerMotor.setTargetPositionRotation2d(optimizedState.angle);
        m_driveMotor.setTargetVelocityMetersPerSecond(optimizedState.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
            m_driveMotor.getVelocityMetersPerSecond(), m_steerMotor.getPositionRotation2d()
        );
        return state;
    }

    public String getDescription() {
        SwerveModuleState state = getState();

        String description = "Loc " + location + ": ";
        description += "angle (deg)=" + state.angle.getDegrees() + " ";
        description += "v (m/s)=" + state.speedMetersPerSecond + " ";
        return description;
    }
}
