package frc.robot.subsystems.swerve.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;

public class SparkMaxDriveMotor implements DriveMotor {
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_PIDController;

    private static final boolean INVERT_MOTOR = false;      // TODO: verify inversion.
    private static final int CURRENT_LIMIT_AMPS = 13;       // TODO: find current limit.

    // SmartMotion and PID.
    private static final int SMART_MOTION_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 0.75);    // TODO: tune PIDs.

    private static final double MAX_VELOCITY_RPM = 5_676;       // TODO: verify max velocity and accel.
    private static final double MIN_VELOCITY_RPM = 0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = 30_000;
    private static final double ALLOWED_ERROR_MOTOR_ROTATIONS = 0.1;

    // Physical.
    private static final double WHEEL_RADIUS_METERS = 0.2;      // TODO: find wheel radius.

    public SparkMaxDriveMotor(int CAN_ID) {
        m_motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(INVERT_MOTOR);
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Coast b/c brake will tip robot.
        m_motor.setIdleMode(IdleMode.kCoast);

        // Configure PID.
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(PID_GAINS.kP, SMART_MOTION_SLOT);
        m_PIDController.setI(PID_GAINS.kI, SMART_MOTION_SLOT);
        m_PIDController.setD(PID_GAINS.kD, SMART_MOTION_SLOT);
        m_PIDController.setIZone(PID_GAINS.kIzone, SMART_MOTION_SLOT);
        m_PIDController.setFF(PID_GAINS.kF, SMART_MOTION_SLOT);
        m_PIDController.setOutputRange(-PID_GAINS.kPeakOutput, PID_GAINS.kPeakOutput, SMART_MOTION_SLOT);

        // Condigure smart motion.
        m_PIDController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
        m_PIDController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, SMART_MOTION_SLOT);
        m_PIDController.setSmartMotionMaxAccel(MAX_ACCELERATION_RPM_PER_SEC, SMART_MOTION_SLOT);
        m_PIDController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR_MOTOR_ROTATIONS, SMART_MOTION_SLOT);
    }

    public void setTargetVelocity(double velocityMetersPerSecond) {
        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        m_PIDController.setReference(velocityRPM, CANSparkMax.ControlType.kVelocity);
    }
}
