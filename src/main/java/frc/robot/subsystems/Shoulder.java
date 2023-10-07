package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;


public class Shoulder extends SubsystemBase implements Loggable {
    // Angle at which the manget detects the sensor.
    public static final double MAGNET_SENSOR_ANGLE_DEGREES = 0.0;
    
    public static final double TARGET_ANGLE_TOLERANCE_DEGREES = 1.0;

    private static Shoulder instance = null;

    private static final double MAX_VOLTAGE = 12.0;
    private static final double GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (74.0 / 16.0);
    
    private static final boolean INVERT_FOLLOWER_MOTOR = true;

    private static final Gains PID_GAINS = new Gains(0.00005, 0.0, 0.0, 0.0001, 0.0, 1.0);
    private static final int PID_SLOT = 0;

    private static final double MAX_VELOCITY_RPM = 5_600.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCEL_RPM_PER_SEC = MAX_VELOCITY_RPM * 4.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.5 / 360.0;
    
    private final CANSparkMax m_motor;
    private final CANSparkMax m_followerMotor;

    private final SparkMaxPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    private final DigitalInput m_zeroSensor;

    public static Shoulder getInstance() {
        if (instance == null) {
            instance = new Shoulder();
        }
        return instance;
    }

    private Shoulder() {
        m_motor = new CANSparkMax(RobotMap.canIDs.Shoulder.MAIN, MotorType.kBrushless);
        m_followerMotor = new CANSparkMax(RobotMap.canIDs.Shoulder.FOLLOWER, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_followerMotor.restoreFactoryDefaults();

        m_followerMotor.follow(m_motor, INVERT_FOLLOWER_MOTOR);

        m_motor.setIdleMode(IdleMode.kCoast);
        m_followerMotor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        setEncoderPositionDegrees(0.0);

        PID_GAINS.setGains(m_pidController, PID_SLOT);
        Gains.configSmartMotion(
            m_pidController,
            MAX_VELOCITY_RPM,
            MIN_VELOCITY_RPM,
            MAX_ACCEL_RPM_PER_SEC, 
            ALLOWED_ERROR_ROTATIONS,
            PID_SLOT
        );

        m_zeroSensor = new DigitalInput(RobotMap.DigitalInputIDs.SHOULDER_ZERO_SENSOR);
    }

    // For tuning PIDs.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        gains.setGains(m_pidController, PID_SLOT);
    }

    @Log (name = "Position (deg)")
    public double getPositionDegrees() {
      return Conversions.motorRotationsToDegrees(m_encoder.getPosition(), GEAR_RATIO);
    }

    @Log (name = "Motor velocity (RPM)")
    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void setEncoderPositionDegrees(double degrees) {
        double motorRotations = Conversions.degreesToMotorRotations(degrees, GEAR_RATIO);
        m_encoder.setPosition(motorRotations);
    }

    public void setPercentOutput(double percentOutput) {
        m_pidController.setReference(percentOutput * MAX_VOLTAGE, ControlType.kVoltage);
    }

    public void setTargetPositionDegrees(double degrees) {
        double motorRotations = Conversions.degreesToMotorRotations(degrees, GEAR_RATIO);
        m_pidController.setReference(motorRotations, ControlType.kSmartMotion, PID_SLOT);
    }

    @Log (name = "Zero sensor")
    public boolean getZeroSensorTriggered() {
        return !m_zeroSensor.get();
    }
}
