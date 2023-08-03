package frc.robot.subsystems.motors.spark_max;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class SparkMaxMotor implements Loggable {
    // Physical.
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 4096.0;

    private static final boolean INVERT_MOTOR = false;
    private static final int CURRENT_LIMIT_AMPS = 13;

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.00005, 0.0, 0.0, 0.00017, 0.0, 1.0);

    // Smart motion.
    private static final double MAX_VELOCITY_RPM = 5810.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = 12_000.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.05;

    // Motor and PID controller.
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_PIDController;

    // Encoders.
    private RelativeEncoder m_relEncoder;
    private AbsoluteEncoder m_absEncoder;
  

    public SparkMaxMotor(int canID) {
        // Get motor.
        m_motor = new CANSparkMax(canID, MotorType.kBrushless);

        // Encoders.
        m_relEncoder = m_motor.getEncoder();
        m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

        // Restore defaults.
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Coast b/c brake will tip robot.
        m_motor.setIdleMode(IdleMode.kCoast);

        // Motor inversion.
        configInversion(INVERT_MOTOR);

        // Config PID controller.
        m_PIDController = m_motor.getPIDController();
        configPIDs(PID_GAINS);
        configSmartMotion();
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

    // For tuning PIDs.
    private void configPIDs(double P, double I, double D, double F, double Izone, double peakOutput) {
        Gains gains = new Gains(P, I, D, F, Izone, peakOutput);
        configPIDs(gains);
    }

    private void configSmartMotion() {
        m_PIDController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, PID_SLOT);
        m_PIDController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, PID_SLOT);
        m_PIDController.setSmartMotionMaxAccel(MAX_ACCELERATION_RPM_PER_SEC, PID_SLOT);
        m_PIDController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR_ROTATIONS, PID_SLOT);
    }

    @Log (name="Abs Position Ticks")
    public double getAbsolutePositionTicks() {
        double positionRotations = m_absEncoder.getPosition();
        double positionTicks = Conversions.rotationsToTicks(positionRotations, TICKS_PER_REV);
        return positionTicks;
    }

    @Log (name="Rel Position Ticks")
    public double getRelativePositionTIcks() {
        double positionRotations = m_relEncoder.getPosition();
        double positionTicks = Conversions.rotationsToTicks(positionRotations, TICKS_PER_REV);
        return positionTicks;
    }

    // public void setTargetVelocityMetersPerSecond(double velocityMetersPerSecond) {
    //     double wheelVelocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
    //     double motorVelocityRPM = wheelVelocityRPM * GEAR_RATIO;
    //     setTargetVelocityRPM(motorVelocityRPM);
    // }

    // public void setTargetVelocityRPM(double velocityRPM) {
    //     m_PIDController.setReference(velocityRPM, CANSparkMax.ControlType.kSmartVelocity, PID_SLOT);
    // }

    // public double getVelocityMetersPerSecond() {
    //     double motorVelocityMetersPerSecond = Conversions.rpmToMetersPerSecond(
    //         m_encoder.getVelocity(), WHEEL_RADIUS_METERS
    //     );
    //     double wheelVelocityMetersPerSecond = motorVelocityMetersPerSecond / GEAR_RATIO;
    //     return wheelVelocityMetersPerSecond;
    // }
}
