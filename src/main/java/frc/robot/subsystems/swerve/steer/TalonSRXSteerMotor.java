package frc.robot.subsystems.swerve.steer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TalonSRXSteerMotor implements SteerMotor, Loggable {
    // Current limits.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 10.0;
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 20.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.3;

    // Motor settings.
    private static final double PERCENT_DEADBAND = 0.001;
    private static final boolean INVERT_SENSOR_PHASE = true;
    private static final boolean INVERT_MOTOR = false;

    // PID.
    private static final int K_PID_LOOP = 0;
    private static final int K_PID_SLOT = 0;
    private static final int K_TIMEOUT_MS = 10;

    // Conversion constants.
    private static final double TICKS_PER_REV = 4096.0;
    private static final double GEAR_RATIO = 1.0;

    // Motion magic.
    private static final Gains PID_GAINS = new Gains(2.8, 0.0, 0.0, 0.5, 0.0, 1.0);
    private static final double MAX_ACCELERATION = 3_000;
    private static final double MAX_VELOCITY = 700;

    // Instance variables.
    private TalonSRX m_motor;

    public TalonSRXSteerMotor(int location) {
        m_motor = new TalonSRX(Constants.SwerveModules.CAN_IDs.STEER[location]);

        // Reset to factory default.
        m_motor.configFactoryDefault();

        // Brake for precise angle.
        m_motor.setNeutralMode(NeutralMode.Brake);

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        configRelativeSensor();

        // Set the relative encoder to start at the inital wheel position.
        // double wheelPositionTicks = -1.0 * (getAbsolutePositionTicks() + Constants.SwerveModules.MINI.WHEEL_ZERO_OFFSET_TICKS[location]);
        double wheelPositionTicks = -1.0 * (getAbsolutePositionTicks() + Constants.SwerveModules.GO_CART.WHEEL_ZERO_OFFSET_TICKS[location]);
        m_motor.setSelectedSensorPosition(wheelPositionTicks % TICKS_PER_REV, K_PID_LOOP, K_TIMEOUT_MS);
    }

    private void configRelativeSensor() {
        // Select which sensor to configure.
        m_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, K_PID_LOOP, K_TIMEOUT_MS);

        // Set deadband to minimum.
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);

        configInversion(INVERT_SENSOR_PHASE, INVERT_MOTOR);
        configPIDs(PID_GAINS);
        configMotionMagic(MAX_ACCELERATION, MAX_VELOCITY);
    }
    
    private void configInversion(boolean invertSensorPhase, boolean invertMotor) {
        m_motor.setSensorPhase(invertSensorPhase);
        m_motor.setInverted(invertMotor);
    }

    private void configPIDs(Gains gains) {
        // Set peak (max) and nominal (min) outputs.
        m_motor.configNominalOutputForward(0, K_TIMEOUT_MS);
        m_motor.configNominalOutputReverse(0, K_TIMEOUT_MS);
        m_motor.configPeakOutputForward(gains.kPeakOutput, K_TIMEOUT_MS);
        m_motor.configPeakOutputReverse(-1.0 * gains.kPeakOutput, K_TIMEOUT_MS);

        // Set gains.
        m_motor.selectProfileSlot(K_PID_SLOT, K_PID_LOOP);
        m_motor.config_kF(K_PID_SLOT, gains.kF, K_TIMEOUT_MS);
        m_motor.config_kP(K_PID_SLOT, gains.kP, K_TIMEOUT_MS);
        m_motor.config_kI(K_PID_SLOT, gains.kI, K_TIMEOUT_MS);
        m_motor.config_kD(K_PID_SLOT, gains.kD, K_TIMEOUT_MS);
        m_motor.config_IntegralZone(K_PID_SLOT, gains.kIzone, K_TIMEOUT_MS);
    }

    // For PID tuning.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        configPIDs(gains);
    }

    private void configMotionMagic(double maxAcceleration, double maxVelocity) {
        m_motor.configMotionAcceleration(maxAcceleration);
        m_motor.configMotionCruiseVelocity(maxVelocity);
    }

    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    public double getRelativePositionTicks() {
        return m_motor.getSelectedSensorPosition(K_PID_LOOP);
    }

    public double getAbsolutePositionTicks() {
        return m_motor.getSensorCollection().getPulseWidthPosition();
    }

    public double getPositionDegrees() {
        double degrees = Conversions.ticksToDegrees(getRelativePositionTicks(), TICKS_PER_REV, GEAR_RATIO);
        return degrees;
    }
    
    public void setTargetPositionDegrees(double targetPositionDegrees) {
        double targetPositionTicks = Conversions.degreesToTicks(targetPositionDegrees, TICKS_PER_REV, GEAR_RATIO);
        setTargetPositionTicks(targetPositionTicks);
    }

    public void setTargetPositionTicks(double targetPositionTicks) {
        m_motor.set(TalonSRXControlMode.MotionMagic, targetPositionTicks);
    }
}
