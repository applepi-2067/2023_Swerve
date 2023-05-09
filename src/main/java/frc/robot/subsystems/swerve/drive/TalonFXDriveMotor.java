package frc.robot.subsystems.swerve.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;

public class TalonFXDriveMotor implements DriveMotor {
    private WPI_TalonFX m_motor;

    // Current limits.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 40.0;       // TODO: find current limits.
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 60.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.5;

    // Motor settings.
    private static final double PERCENT_DEADBAND = 0.001;
    private static final boolean INVERT_MOTOR = false;     // TODO: verify inversion.

    // PID.
    private static final int K_PID_LOOP = 0;
    private static final int K_PID_SLOT = 0;
    private static final int K_TIMEOUT_MS = 10;
    private static final Gains PID_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 1.0);             // TODO: tune PIDs.

    // Conversion constants.
    private static final double TICKS_PER_REV = 2048.0;
    private static final double WHEEL_RADIUS_METERS = 0.2;      // TODO: find wheel radius.

    public TalonFXDriveMotor(int location) {
        m_motor = new WPI_TalonFX(Constants.SwerveModules.CAN_IDs.DRIVE[location]);

        // Reset to default configuration.
        m_motor.configFactoryDefault();

        // Set neutral to coast (avoid tipping).
        m_motor.setNeutralMode(NeutralMode.Coast);

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        configVelocityControl();
    }

    private void configVelocityControl() {
        // Select integrated sensor to configure.
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, K_PID_LOOP, K_TIMEOUT_MS);

        // Set deadband to minimum.
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);

        // Motor inversion.
        m_motor.setInverted(INVERT_MOTOR);

        // Set peak (max) and nominal (min) outputs.
        m_motor.configNominalOutputForward(0, K_TIMEOUT_MS);
        m_motor.configNominalOutputReverse(0, K_TIMEOUT_MS);
        m_motor.configPeakOutputForward(PID_GAINS.kPeakOutput, K_TIMEOUT_MS);
        m_motor.configPeakOutputReverse(-1.0 * PID_GAINS.kPeakOutput, K_TIMEOUT_MS);

        // Set gains.
        m_motor.selectProfileSlot(K_PID_SLOT, K_PID_LOOP);
        m_motor.config_kF(K_PID_SLOT, PID_GAINS.kF, K_TIMEOUT_MS);
        m_motor.config_kP(K_PID_SLOT, PID_GAINS.kP, K_TIMEOUT_MS);
        m_motor.config_kI(K_PID_SLOT, PID_GAINS.kI, K_TIMEOUT_MS);
        m_motor.config_kD(K_PID_SLOT, PID_GAINS.kD, K_TIMEOUT_MS);
        m_motor.config_IntegralZone(K_PID_SLOT, PID_GAINS.kIzone, K_TIMEOUT_MS);
    }

    public void setTargetVelocityMetersPerSecond(double velocityMetersPerSecond) {
        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        double velocityTicksPer100MS = Conversions.RPMToTicksPer100MS(velocityRPM, TICKS_PER_REV);

        m_motor.set(TalonFXControlMode.Velocity, velocityTicksPer100MS);
    }
}
