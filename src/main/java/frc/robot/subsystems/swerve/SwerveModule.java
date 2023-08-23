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

        // Get closest target angle.
        Rotation2d targetPositionRotation2d = getTargetPositionRotation2d(optimizedState.angle, m_steerMotor.getPositionRotation2d());

        // Set steer and drive motors to targets.
        m_steerMotor.setTargetPositionRotation2d(targetPositionRotation2d);
        m_driveMotor.setTargetVelocityMetersPerSecond(optimizedState.speedMetersPerSecond);
    }

    public Rotation2d getTargetPositionRotation2d(Rotation2d optimizedStateRotation2d, Rotation2d currPositionRotation2d) {
        // NOTE: WPILib implementation return angle on (-pi, pi). Our job to figure out closest to curr rotation.
        // Get the rotation WPILib considers the curr position.
        Rotation2d wpiCurrPositionRotation2d = currPositionRotation2d.plus(new Rotation2d());

        // Find curr to target delta.
        Rotation2d positionDeltaRotation2d = optimizedStateRotation2d.minus(wpiCurrPositionRotation2d);

        // Add delta to actual curr rotation. Remember to use rotations or WPILib will give ans on (-pi, pi).
        double targetPositionRotations = currPositionRotation2d.getRotations() + positionDeltaRotation2d.getRotations();
        Rotation2d targetPositionRotation2d = Rotation2d.fromRotations(targetPositionRotations);
        
        return targetPositionRotation2d;
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
