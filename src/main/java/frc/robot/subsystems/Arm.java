package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase implements Loggable {
    private static final int CURRENT_LIMIT_AMPS = 30;
    private static final boolean INVERT_MOTOR = true;
    private static final double MAX_VOLTAGE = 12.0;

    private static final double GEAR_RATIO = (84.0 / 29.0) * (76.0 / 21.0);
    private static final double OUTPUT_SPROCKET_PITCH_DIAMETERS_METERS = 0.020574;
    private static final double RIGGING_EXTENSION_RATIO = 2.0;
    private static final double METERS_PER_REV = (Math.PI * OUTPUT_SPROCKET_PITCH_DIAMETERS_METERS) * RIGGING_EXTENSION_RATIO / GEAR_RATIO;

    private static final int PID_SLOT = 0;
    private static final Gains GAINS = new Gains(3.5e-5, 0.0, 0.0, 5e-5, 0.0, 1.0);
    
    private static final double MAX_VELOCITY_RPM = 11_000.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCEL_RPM_PER_SEC = MAX_VELOCITY_RPM * 4.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.05;

    private static Arm instance = null;

    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_pidController;
    private final RelativeEncoder m_encoder;
    private final DigitalInput m_zeroSensor;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        m_motor = new CANSparkMax(Constants.canIDs.ARM, MotorType.kBrushless);
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        setEncoderPosition(0.0);

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
        m_motor.setInverted(INVERT_MOTOR);
        m_motor.setIdleMode(IdleMode.kBrake);

        GAINS.setGains(m_pidController, PID_SLOT);
        Gains.configSmartMotion(
            m_pidController,
            MAX_VELOCITY_RPM,
            MIN_VELOCITY_RPM,
            MAX_ACCEL_RPM_PER_SEC,
            ALLOWED_ERROR_ROTATIONS,
            PID_SLOT
        );

        m_zeroSensor = new DigitalInput(Constants.DigitalInputIDs.ARM_ZERO_SENSOR);
    }

    // For tuning PIDs.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        gains.setGains(m_pidController, PID_SLOT);
    }

    private double metersToMotorRotations(double meters) {
        return meters / METERS_PER_REV;
    }

    private double motorRotationsToMeters(double rotations) {
        return rotations * METERS_PER_REV;
    }

    @Log (name = "Position (rotations)")
    public double getPositionRotations() {
      return m_encoder.getPosition();
    }

    @Log (name = "Position (meters)")
    public double getPositionMeters() {
        return motorRotationsToMeters(m_encoder.getPosition());
    }

    @Log (name = "Motor velocity (RPM)")
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void setEncoderPosition(double meters) {
        m_encoder.setPosition(metersToMotorRotations(meters));
    }

    public void setPercentOutput(double percentOutput) {
        m_pidController.setReference(percentOutput * MAX_VOLTAGE, ControlType.kVoltage);
    }

    public void setTargetPosition(double meters) {
        m_pidController.setReference(metersToMotorRotations(meters), ControlType.kSmartMotion);
    }

    @Log (name = "Zero sensor")
    public boolean getZeroSensorTriggered() {
        return !m_zeroSensor.get();
    }
}
