package frc.robot.subsystems.swerve.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SparkMaxDriveMotor implements DriveMotor, Loggable {
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_PIDController;
    private RelativeEncoder m_encoder;

    private static final boolean INVERT_MOTOR = false;      // TODO: verify inversion.
    private static final int CURRENT_LIMIT_AMPS = 13;       // TODO: find current limit.

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.6, 0.0, 0.0, 0.7, 0.0, 1.0);    // TODO: tune PIDs.

    // Physical.
    private static final double WHEEL_RADIUS_METERS = 0.2;      // TODO: find wheel radius.

    public SparkMaxDriveMotor(int location) {
        m_motor = new CANSparkMax(Constants.SwerveModules.CAN_IDs.DRIVE[location], MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Coast b/c brake will tip robot.
        m_motor.setIdleMode(IdleMode.kCoast);

        configInversion(INVERT_MOTOR);

        m_PIDController = m_motor.getPIDController();
        configPIDs(PID_GAINS);
    }

    private void configInversion(boolean invertMotor) {
        m_motor.setInverted(invertMotor);
    }

    private void configPIDs(Gains gains) {
        m_PIDController.setP(gains.kP, PID_SLOT);
        m_PIDController.setI(gains.kI, PID_SLOT);
        m_PIDController.setD(gains.kD, PID_SLOT);
        m_PIDController.setFF(gains.kF, PID_SLOT); 
        m_PIDController.setIZone(gains.kIzone, PID_SLOT);
        m_PIDController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, PID_SLOT);
    }

    @Config     // ONLY FOR TUNING PIDs
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        configPIDs(gains);
    }

    public void setTargetVelocityMetersPerSecond(double velocityMetersPerSecond) {
        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        m_PIDController.setReference(velocityRPM, CANSparkMax.ControlType.kVelocity);
    }

    @Log (name="Drive Motor Velocity (Meters Per Second)")
    public double getMotorVelocityMetersPerSecond() {
        double velocityMetersPerSecond = Conversions.RPM_ToMetersPerSecond(
            m_encoder.getVelocity(), WHEEL_RADIUS_METERS
        );
        return velocityMetersPerSecond;
    }
}
