package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Shoulder extends SubsystemBase{
    private static Shoulder instance = null;

    public final int CAN_ID = 5;
    public final int FOLLOWER_CAN_ID = 6;
    
    private final boolean INVERT_MOTOR = false;

    private final CANSparkMax m_motor;
    // private final CANSparkMax m_followerMotor;

    private final SparkMaxPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    public static Shoulder getInstance() {
        if (instance == null) {
            instance = new Shoulder();
        }
        return instance;
    }

    private Shoulder() {
        m_motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        // m_followerMotor = new CANSparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        // m_followerMotor.restoreFactoryDefaults();

        // m_followerMotor.follow(m_motor);

        m_motor.setInverted(INVERT_MOTOR);
        // m_followerMotor.setInverted(!INVERT_MOTOR);

        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
    }

    @Log (name = "Position (deg)")
    public double getPosition() {
      return m_encoder.getPosition();
    }

    public void setPercentOutput(double percentOutput) {
        m_pidController.setReference(percentOutput / 3.0, ControlType.kVoltage);
    }
}
