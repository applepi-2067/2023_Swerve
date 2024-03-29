package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.RobotMap;


public class SwerveModule {
    // Reported abs encoder position at wheel zero.
    private static final double[] STEER_WHEEL_ZERO_OFFSET_DEGREES = {240.52, 157.41, 223.30, 27.93};

    private final DriveMotor m_driveMotor;
    private final SteerMotor m_steerMotor;

    private final int location;

    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
        m_driveMotor = new DriveMotor(RobotMap.canIDs.Drivetrain.DRIVE[location]);
        m_steerMotor = new SteerMotor(
            RobotMap.canIDs.Drivetrain.STEER[location],
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

        if (Math.abs(positionDeltaDegrees) > 90.0) {
            // Case: delta(target=135, curr=0) = -45.
            positionDeltaDegrees += 180.0 * Math.signum(positionDeltaDegrees);
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

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            m_driveMotor.getPositionMeters(),
            m_steerMotor.getPositionRotation2d()
        );
        return position;
    }

    public String getDescription() {
        SwerveModuleState state = getState();

        String description = "Loc " + location + ": ";
        description += "angle (deg)=" + state.angle.getDegrees() + " ";
        description += "v (m/s)=" + state.speedMetersPerSecond + " ";
        description += "distance (meters)=" + m_driveMotor.getPositionMeters() + " ";
        return description;
    }
}
