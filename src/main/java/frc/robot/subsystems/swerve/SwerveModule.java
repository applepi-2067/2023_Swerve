package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
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
    private SparkMaxSteerMotor m_steerMotor;

    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
        m_driveMotor = new TalonFXDriveMotor(CAN_IDS.DRIVE[location]);
        m_steerMotor = new SparkMaxSteerMotor(
            CAN_IDS.STEER[location],
            STEER_WHEEL_ZERO_OFFSET_DEGREES[location]
        );
    }

    public void setTargetState(double velocityMetersPerSecond, double targetPositionDegrees) {
        setTargetState(new SwerveModuleState(velocityMetersPerSecond, Rotation2d.fromDegrees(targetPositionDegrees)));
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Optimize state.
        double currPositionDegrees = m_steerMotor.getPositionRotation2d().getDegrees();

        double velocityMetersPerSecond = targetState.speedMetersPerSecond;
        double targetPositionDegrees = targetState.angle.getDegrees();
        if (targetPositionDegrees < 0.0) {
            targetPositionDegrees += 360.0;
        }

        double positionDeltaDegrees = targetPositionDegrees - currPositionDegrees;

        if (Math.abs(positionDeltaDegrees) > 180.0) {
            // Case: delta(target=359, curr=1) = 2.
            positionDeltaDegrees -= 360.0 * Math.signum(positionDeltaDegrees);
        }

        if (Math.abs(positionDeltaDegrees) > 90.0) {
            // Case: delta(target=135, curr=0) = -45.
            positionDeltaDegrees -= 180.0 * Math.signum(positionDeltaDegrees);
            velocityMetersPerSecond *= -1.0;
        }

        targetPositionDegrees = currPositionDegrees + positionDeltaDegrees;
        Rotation2d targetPositionRotation2d = Rotation2d.fromDegrees(targetPositionDegrees);

        // Set steer and drive motors to targets.
        m_steerMotor.setTargetPositionRotation2d(targetPositionRotation2d);
        m_driveMotor.setTargetVelocityMetersPerSecond(velocityMetersPerSecond);
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
