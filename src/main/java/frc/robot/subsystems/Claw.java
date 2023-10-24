package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Claw extends SubsystemBase implements Loggable {
  private static final boolean INVERT_MOTOR = true;

  private static Claw instance = null;

  private final Solenoid m_solenoid;
  private final TalonSRX m_motor;
  private final DigitalInput m_irSensor;

  public static Claw getInstance() {
    if (instance == null) {
      instance = new Claw();
    }
    return instance;
  }

  public Claw() {
    m_solenoid = new Solenoid(
      RobotMap.canIDs.Claw.CLAW_GRASP_PNEUMATICS_ID,
      PneumaticsModuleType.CTREPCM,
      RobotMap.canIDs.Claw.CLAW_GRASP_PNEUMATICS_CHANNEL
    );

    m_motor = new TalonSRX(RobotMap.canIDs.Claw.CLAW_BELT);
    m_motor.setInverted(INVERT_MOTOR);
    m_motor.setNeutralMode(NeutralMode.Coast);

    m_irSensor = new DigitalInput(RobotMap.DigitalInputIDs.CLAW_IR_SENSOR);
  }
  
  public void openClaw() {
    m_solenoid.set(true);
  }

  public void closeClaw() {
    m_solenoid.set(false);
  }

  public void setPercentOutput(double percentOutput) {
    m_motor.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }
  
  @Log (name = "IR Sensor")
  public boolean getIRSensorTriggered() {
    return !m_irSensor.get();
  }
}
