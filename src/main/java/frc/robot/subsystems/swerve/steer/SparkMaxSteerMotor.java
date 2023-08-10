package frc.robot.subsystems.swerve.steer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class SparkMaxSteerMotor implements SteerMotor, Loggable {
    // TODO: Find physical limits.
    // TODO: Tune PIDs.
    // TODO: Find wheel zero offsets.

    // Gear ratio: 3:1, 4:1, belt.
    private static final double GEAR_RATIO = (84.0 / 29.0) * (76.0 / 21.0) * (66.0 / 15.0);

    // Physical.
    private static final boolean INVERT_MOTOR = false;
    private static final int CURRENT_LIMIT_AMPS = 13;

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.00005, 0.0, 0.0, 0.00017, 0.0, 1.0);

    // Smart motion.
    private static final double MAX_VELOCITY_RPM = 5810.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = 12_000.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.05;

    // Motor and PID controller.
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_PIDController;

    // Encoders.
    private RelativeEncoder m_relEncoder;
    private AbsoluteEncoder m_absEncoder;
  

    public SparkMaxSteerMotor(int canID, double wheelZeroOffsetDegrees) {
        m_motor = new CANSparkMax(canID, MotorType.kBrushless);

        // Encoders.
        m_relEncoder = m_motor.getEncoder();
        m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

        // Restore defaults.
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Set idle brake mode for accurate steering.
        m_motor.setIdleMode(IdleMode.kBrake);

        // Motor inversion.
        m_motor.setInverted(INVERT_MOTOR);

        // Config PID controller.
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setFeedbackDevice(m_relEncoder);

        // Config PIDs and smart motion.
        configPIDs(PID_GAINS);
        configSmartMotion();

        // Match relative encoder to wheel position.
        double wheelPositionRotations = m_absEncoder.getPosition() - Conversions.degreesToRotations(wheelZeroOffsetDegrees);
        m_relEncoder.setPosition(wheelPositionRotations);
    }

    private void configPIDs(Gains gains) {
        m_PIDController.setP(gains.kP, PID_SLOT);
        m_PIDController.setI(gains.kI, PID_SLOT);
        m_PIDController.setD(gains.kD, PID_SLOT);
        m_PIDController.setFF(gains.kF, PID_SLOT); 
        m_PIDController.setIZone(gains.kIzone, PID_SLOT);
        m_PIDController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, PID_SLOT);
    }

    // For tuning PIDs.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        configPIDs(gains);
    }

    private void configSmartMotion() {
        m_PIDController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, PID_SLOT);
        m_PIDController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, PID_SLOT);
        m_PIDController.setSmartMotionMaxAccel(MAX_ACCELERATION_RPM_PER_SEC, PID_SLOT);
        m_PIDController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR_ROTATIONS, PID_SLOT);
    }

    @Log (name="Rel pos degrees")
    public double getPositionDegrees() {
        // Relative position.
        double positionRotations = m_relEncoder.getPosition();
        double positionDegrees = Conversions.rotationsToDegrees(positionRotations);
        return positionDegrees;
    }

    @Log (name="Abs pos degrees")
    public double getAbsolutePositionTicks() {
        // NOTE: Encoder reads wheel 1:1.
        double positionRotations = m_absEncoder.getPosition();
        double positionDegrees = Conversions.rotationsToDegrees(positionRotations);
        return positionDegrees;
    }

    public void setTargetPositionDegrees(double targetPositionDegrees) {
        double targetPositionRotations = Conversions.degreesToRotations(targetPositionDegrees);
        m_PIDController.setReference(targetPositionRotations, ControlType.kPosition);
    }
}
