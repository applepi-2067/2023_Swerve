package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants;
import frc.robot.utils.Gains;


public class Shoulder extends SubsystemBase implements Loggable {
    private static Shoulder instance = null;

    // TODO: Set max voltage.
    private static final double MAX_VOLTAGE = 12.0;
    private static final double GEAR_RATIO = 5.0 * 5.0;
    
    private static final boolean INVERT_FOLLOWER_MOTOR = true;

    private static final Gains PID_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    private static final int PID_SLOT = 0;
    
    private final CANSparkMax m_motor;
    private final CANSparkMax m_followerMotor;

    private final SparkMaxPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    public static Shoulder getInstance() {
        if (instance == null) {
            instance = new Shoulder();
        }
        return instance;
    }

    private Shoulder() {
        m_motor = new CANSparkMax(Constants.CAN_IDs.Shoulder.MAIN, MotorType.kBrushless);
        m_followerMotor = new CANSparkMax(Constants.CAN_IDs.Shoulder.FOLLOWER, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_followerMotor.restoreFactoryDefaults();

        m_followerMotor.follow(m_motor, INVERT_FOLLOWER_MOTOR);

        m_motor.setIdleMode(IdleMode.kCoast);
        m_followerMotor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.0);

        // Set PIDs.
        m_pidController.setP(PID_GAINS.kP, PID_SLOT);
        m_pidController.setI(PID_GAINS.kI, PID_SLOT);
        m_pidController.setD(PID_GAINS.kD, PID_SLOT);
        m_pidController.setIZone(PID_GAINS.kIzone, PID_SLOT);
        m_pidController.setFF(PID_GAINS.kF, PID_SLOT);
        m_pidController.setOutputRange(-1.0 * PID_GAINS.kPeakOutput, PID_GAINS.kPeakOutput, PID_SLOT);
    }

    @Log (name = "Position (deg)")
    public double getPosition() {
      return (m_encoder.getPosition() / GEAR_RATIO) * 360.0;
    }

    public void setPercentOutput(double percentOutput) {
        m_pidController.setReference(percentOutput * MAX_VOLTAGE, ControlType.kVoltage);
    }

    public void setPosition(double degrees) {
        double motorRotations = (degrees / 360.0) * GEAR_RATIO;
        m_pidController.setReference(motorRotations, ControlType.kPosition, PID_SLOT);
    }
}
