package frc.robot.subsystems.swerve.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class TalonFXDriveMotor implements DriveMotor, Loggable {
    // TODO: Find physical limits.

    private WPI_TalonFX m_motor;

    // Current limits.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 40.0;
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 60.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.5;

    // Motor settings.
    private static final double PERCENT_DEADBAND = 0.001;
    private static final boolean INVERT_MOTOR = false;

    // PID.
    private static final int K_PID_LOOP = 0;
    private static final int K_PID_SLOT = 0;
    private static final int K_TIMEOUT_MS = 10;
    private static final Gains PID_GAINS = new Gains(0.025, 0.0, 0.0, 0.045, 0.0, 1.0);

    // Conversion constants.
    private static final double TICKS_PER_REV = 2048.0;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0 / 2.0);

    // Gear Ratio: motor pinion -> gear, bevel gear pair.
    private static final double GEAR_RATIO = (30.0 / 14.0) * (45.0 / 15.0);

    public TalonFXDriveMotor(int canID) {
        m_motor = new WPI_TalonFX(canID);

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

        // Set gains.
        PID_GAINS.setGains(m_motor, K_PID_SLOT, K_PID_LOOP, K_TIMEOUT_MS);
    }

    public void setTargetVelocityMetersPerSecond(double wheelVelocityMetersPerSecond) {
        double velocityMetersPerSecond = wheelVelocityMetersPerSecond * GEAR_RATIO;

        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        double velocityTicksPer100ms = Conversions.RPMToTicksPer100ms(velocityRPM, TICKS_PER_REV);
        m_motor.set(TalonFXControlMode.Velocity, velocityTicksPer100ms);
    }

    @Log (name="Velocity (m/s)")
    public double getVelocityMetersPerSecond() {
        double velocityTicksPer100ms = m_motor.getSelectedSensorVelocity(K_PID_LOOP);
        double velocityRPM = Conversions.ticksPer100msToRPM(velocityTicksPer100ms, TICKS_PER_REV);
        double velocityMetersPerSecond = Conversions.rpmToMetersPerSecond(velocityRPM, WHEEL_RADIUS_METERS);

        double wheelVelocityMetersPerSecond = velocityMetersPerSecond / GEAR_RATIO;
        return wheelVelocityMetersPerSecond;
    }
}
