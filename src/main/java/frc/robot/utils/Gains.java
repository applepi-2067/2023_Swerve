package frc.robot.utils;

import com.revrobotics.SparkMaxPIDController;


public class Gains {
    public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final double kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
    }

	public void setGains(SparkMaxPIDController pidController, int pidSlot) {
		pidController.setP(kP, pidSlot);
        pidController.setI(kI, pidSlot);
        pidController.setD(kD, pidSlot);
        pidController.setFF(kF, pidSlot); 
        pidController.setIZone(kIzone, pidSlot);
        pidController.setOutputRange(-kPeakOutput, kPeakOutput, pidSlot);
	}
}
