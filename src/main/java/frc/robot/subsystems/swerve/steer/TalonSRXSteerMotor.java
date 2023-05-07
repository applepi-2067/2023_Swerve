package frc.robot.subsystems.swerve.steer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.annotations.Log;

public class TalonSRXSteerMotor implements SteerMotor {
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
    private static final double GEAR_RATIO = 1.0;

    // Motion magic.
    private static final Gains PID_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);             // TODO: tune PIDs.
    private static final double MAX_ACCELERATION_TICKS_PER_100MS_PER_SECOND = Conversions.RPMToTicksPer100MS(60.0, TICKS_PER_REV);      // TODO: find max accel and velocity.
    private static final double MAX_VELOCITY_TICKS_PER_100MS = Conversions.RPMToTicksPer100MS(60.0, TICKS_PER_REV);

    // Offset from motor absolute zero to wheel zero.
    private static final double WHEEL_ZERO_OFFSET_TICKS = Conversions.degreesToTicks(10.0, TICKS_PER_REV, GEAR_RATIO);       // TODO: Find wheel zero offset.

    public TalonSRXSteerMotor(int CAN_ID) {
        m_motor = new TalonSRX(CAN_ID);

        // Brake for precise angle.
        m_motor.setNeutralMode(NeutralMode.Brake);

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        configRelativeSensor(PID_GAINS, INVERT_SENSOR_PHASE, INVERT_MOTOR);
    }
    
    private double getInitialWheelPositionTicks() {
        double initalAbsoluteEncoderPositionTicks = m_motor.getSensorCollection().getPulseWidthPosition();
        return initalAbsoluteEncoderPositionTicks + WHEEL_ZERO_OFFSET_TICKS;
    }

    public void configRelativeSensor(Gains PID_Gains, boolean invertSensorPhase, boolean invertMotor) {
        // Select which sensor to configure.
        m_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, K_PID_LOOP, K_TIMEOUT_MS);

        // Set deadband to minimum.
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);

        // Motor inversion.
        m_motor.setSensorPhase(invertSensorPhase);
        m_motor.setInverted(invertMotor);

        // Set peak (max) and nominal (min) outputs.
        m_motor.configNominalOutputForward(0, K_TIMEOUT_MS);
        m_motor.configNominalOutputReverse(0, K_TIMEOUT_MS);
        m_motor.configPeakOutputForward(PID_Gains.kPeakOutput, K_TIMEOUT_MS);
        m_motor.configPeakOutputReverse(-1.0 * PID_Gains.kPeakOutput, K_TIMEOUT_MS);

        // Set gains.
        m_motor.selectProfileSlot(K_PID_SLOT, K_PID_LOOP);
        m_motor.config_kF(K_PID_SLOT, PID_Gains.kF, K_TIMEOUT_MS);
        m_motor.config_kP(K_PID_SLOT, PID_Gains.kP, K_TIMEOUT_MS);
        m_motor.config_kI(K_PID_SLOT, PID_Gains.kI, K_TIMEOUT_MS);
        m_motor.config_kD(K_PID_SLOT, PID_Gains.kD, K_TIMEOUT_MS);
        m_motor.config_IntegralZone(K_PID_SLOT, PID_Gains.kIzone, K_TIMEOUT_MS);
    
        // Set max acceleration and velocity.
        m_motor.configMotionAcceleration(MAX_ACCELERATION_TICKS_PER_100MS_PER_SECOND);
        m_motor.configMotionCruiseVelocity(MAX_VELOCITY_TICKS_PER_100MS);

        // Set the relative encoder to start at the inital wheel position.
        m_motor.setSelectedSensorPosition(getInitialWheelPositionTicks(), K_PID_LOOP, K_TIMEOUT_MS);
    }

    @Log (name="Position Ticks")
    public double getPositionTicks() {
        return m_motor.getSelectedSensorPosition(K_PID_LOOP);
    }

    @Log (name="Position Degrees")
    public double getPositionDegrees() {
        double rawDegrees = Conversions.ticksToDegrees(getPositionTicks(), TICKS_PER_REV, GEAR_RATIO);
        double degrees = Conversions.shiftHalfCircle(rawDegrees);
        return degrees;
    }

    public void setTargetPositionDegrees(double targetPositionDegrees) {
        double currPositionDegrees = getPositionDegrees();

        // Optimize target angle.
        targetPositionDegrees = Conversions.optimizeTargetAngle(targetPositionDegrees, currPositionDegrees);

        // Find tick change needed.
        double deltaDegrees = targetPositionDegrees - currPositionDegrees;
        double deltaTicks = Conversions.degreesToTicks(deltaDegrees, TICKS_PER_REV, GEAR_RATIO);

        double targetTicks = getPositionTicks() + deltaTicks;
        m_motor.set(TalonSRXControlMode.MotionMagic, targetTicks);

        SmartDashboard.putNumber("Target Position Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Target Position Ticks", targetTicks);
    }
}
