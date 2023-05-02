package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;

public class SteerMotor {
    private TalonSRX m_motor;

    // Current limits.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 10.0;       // TODO: find current limits.
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 20.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.3;

    // Motor settings.
    private static final double PERCENT_DEADBAND = 0.001;
    private static final boolean INVERT_SENSOR_PHASE = false;
    private static final boolean INVERT_MOTOR = false;              // TODO: verify inversion.

    // PID.
    private static final int K_PID_LOOP = 0;
    private static final int K_PID_SLOT = 0;
    private static final int K_TIMEOUT_MS = 10;

    // Conversion constants.
    private static final double TICKS_PER_REV = 4096.0;
    private static final double GEAR_RATIO = (20.0 / 1.0);          // TODO: Find gear ratio.

    // Motion magic.
    private static final Gains PID_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);             // TODO: tune PIDs.
    private static final double MAX_ACCELERATION_TICKS_PER_100MS_PER_SECOND = Conversions.RPMToTicksPer100MS(60.0, TICKS_PER_REV);      // TODO: find max accel and velocity.
    private static final double MAX_VELOCITY_TICKS_PER_100MS = Conversions.RPMToTicksPer100MS(60.0, TICKS_PER_REV);

    public SteerMotor(int CAN_ID) {
        m_motor = new TalonSRX(CAN_ID);

        // Brake for precise angle.
        m_motor.setNeutralMode(NeutralMode.Brake);

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        configAbsoluteSensor();
    }
    
    private void configAbsoluteSensor() {
        // Select which sensor to configure.
        m_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, K_PID_LOOP, K_TIMEOUT_MS);

        // Set deadband to minimum.
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);

        // Motor inversion.
        m_motor.setSensorPhase(INVERT_SENSOR_PHASE);
        m_motor.setInverted(INVERT_MOTOR);

        // Set peak (max) and nominal (min) outputs.
        m_motor.configNominalOutputForward(0, K_TIMEOUT_MS);
        m_motor.configNominalOutputReverse(0, K_TIMEOUT_MS);
        m_motor.configPeakOutputForward(1, K_TIMEOUT_MS);
        m_motor.configPeakOutputReverse(-1, K_TIMEOUT_MS);

        // Set gains.
        m_motor.selectProfileSlot(K_PID_SLOT, K_PID_LOOP);
        m_motor.config_kF(K_PID_SLOT, PID_GAINS.kF, K_TIMEOUT_MS);
        m_motor.config_kP(K_PID_SLOT, PID_GAINS.kP, K_TIMEOUT_MS);
        m_motor.config_kI(K_PID_SLOT, PID_GAINS.kI, K_TIMEOUT_MS);
        m_motor.config_kD(K_PID_SLOT, PID_GAINS.kD, K_TIMEOUT_MS);
        m_motor.config_IntegralZone(K_PID_SLOT, PID_GAINS.kIzone, K_TIMEOUT_MS);
    
        // Set max acceleration and velocity.
        m_motor.configMotionAcceleration(MAX_ACCELERATION_TICKS_PER_100MS_PER_SECOND);
        m_motor.configMotionCruiseVelocity(MAX_VELOCITY_TICKS_PER_100MS);
    }

    public double getPositionDegrees() {
        double positionTicks = m_motor.getSelectedSensorPosition(K_PID_LOOP);
        return Conversions.ticksToDegrees(positionTicks, TICKS_PER_REV, GEAR_RATIO);
    }

    public void setTargetPositionDegrees(double targetDegrees) {
        double targetTicks = Conversions.degreesToTicks(targetDegrees, TICKS_PER_REV, GEAR_RATIO);
        m_motor.set(TalonSRXControlMode.MotionMagic, targetTicks);
    }
}
