package frc.robot.subsystems.swerve.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.utils.Gains;

public class SparkMaxDriveMotor implements DriveMotor {
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_PIDController;

    private static final boolean INVERT_MOTOR = false;      // TODO: verify inversion.
    private static final int CURRENT_LIMIT_AMPS = 13;       // TODO: find current limit.

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 1.0);    // TODO: tune PIDs.

    public SparkMaxDriveMotor(int CAN_ID) {
        m_motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(INVERT_MOTOR);
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Coast b/c brake will tip robot.
        m_motor.setIdleMode(IdleMode.kCoast);

        // Configure PID.
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(PID_GAINS.kP, PID_SLOT);
        m_PIDController.setI(PID_GAINS.kI, PID_SLOT);
        m_PIDController.setD(PID_GAINS.kD, PID_SLOT);
        m_PIDController.setIZone(PID_GAINS.kIzone, PID_SLOT);
        m_PIDController.setFF(PID_GAINS.kF, PID_SLOT);
        m_PIDController.setOutputRange(-PID_GAINS.kPeakOutput, PID_GAINS.kPeakOutput, PID_SLOT);
    }

    public void setTargetPercentOutput(double percentOutput) {
        m_PIDController.setReference(percentOutput, ControlType.kVoltage);
    }
}
