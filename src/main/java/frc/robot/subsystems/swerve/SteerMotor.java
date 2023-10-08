package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;


public class SteerMotor implements Loggable {

    // Gear ratio: 3:1, 4:1, belt.
    private static final double GEAR_RATIO = (84.0 / 29.0) * (76.0 / 21.0) * (66.0 / 15.0);

    // Physical.
    private static final boolean INVERT_MOTOR = false;
    private static final int CURRENT_LIMIT_AMPS = 30;

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(17.0, 0.0, 0.0, 0.12, 0.0, 1.0);

    // Smart motion.
    private static final double MAX_VELOCITY_RPM = 11_000.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = MAX_VELOCITY_RPM * 2.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.05;

    // Motor and PID controller.
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PIDController;

    // Encoders.
    private final AbsoluteEncoder m_absEncoder;

    public SteerMotor(int canID, double wheelZeroOffsetDegrees) {
        m_motor = new CANSparkMax(canID, MotorType.kBrushless);

        // Restore defaults.
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Set idle brake mode for accurate steering.
        m_motor.setIdleMode(IdleMode.kBrake);

        // Motor inversion.
        m_motor.setInverted(INVERT_MOTOR);

        // Abs encoder.
        m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_absEncoder.setZeroOffset(Rotation2d.fromDegrees(wheelZeroOffsetDegrees).getRotations());

        // Config PID controller.
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setFeedbackDevice(m_absEncoder);

        // Set PID wrapping (wrap between 0 and 360 degrees).
        m_PIDController.setPositionPIDWrappingEnabled(true);
        m_PIDController.setPositionPIDWrappingMinInput(0.0);
        m_PIDController.setPositionPIDWrappingMaxInput(1.0);

        // Config PIDs and smart motion.
        PID_GAINS.setGains(m_PIDController, PID_SLOT);
        Gains.configSmartMotion(
            m_PIDController,
            MAX_VELOCITY_RPM,
            MIN_VELOCITY_RPM,
            MAX_ACCELERATION_RPM_PER_SEC, 
            ALLOWED_ERROR_ROTATIONS,
            PID_SLOT
        );
    }

    // For tuning PIDs.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        gains.setGains(m_PIDController, PID_SLOT);
    }

    public Rotation2d getPositionRotation2d() {
        double positionRotations = m_absEncoder.getPosition();
        Rotation2d positionRotation2d = Rotation2d.fromRotations(positionRotations);
        return positionRotation2d;
    }

    public void setTargetPositionRotation2d(Rotation2d targetPositionRotation2d) {
        double targetPositionRotations = targetPositionRotation2d.getRotations();
        m_PIDController.setReference(targetPositionRotations, ControlType.kPosition);
    }
}
