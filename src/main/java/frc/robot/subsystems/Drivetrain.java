// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveModule;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  // Swerve module offsets from center.
  public static final Translation2d[] SWERVE_MODULE_CENTER_OFFSETS = {
    new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(10)),
    new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(-10)),
    new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(10)),
    new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(-10)),
  };

  // TODO: set max speeds.
  // Max speeds.
  private static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 8.0;
  private static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = 8.0;
  
  // Swerve modules.
  private SwerveDriveKinematics m_kinematics;
  private SwerveModule[] m_swerveModules;

  // Gyro.
  private static final int GYRO_CAN_ID = 9;
  private PigeonIMU m_gyro;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    // Swerve drive kinematics.
    m_kinematics = new SwerveDriveKinematics(
      SWERVE_MODULE_CENTER_OFFSETS[0], SWERVE_MODULE_CENTER_OFFSETS[1],
      SWERVE_MODULE_CENTER_OFFSETS[2], SWERVE_MODULE_CENTER_OFFSETS[3]
    );
    
    // Create swerve modules.
    m_swerveModules = new SwerveModule[4];
    for (int location = 0; location < 4; location++) {
      m_swerveModules[location] = new SwerveModule(location);
    }

    // Create and reset gyro.
    TalonSRX gyroController = new TalonSRX(GYRO_CAN_ID);
    m_gyro = new PigeonIMU(gyroController);
    m_gyro.setYaw(0.0);
  }

  // Log state.
  @Log (name="Swerve Module 0")
  public String getSwerveModule0Description() {
    return m_swerveModules[0].getDescription();
  }

  @Log (name="Swerve Module 1")
  public String getSwerveModule1Description() {
    return m_swerveModules[1].getDescription();
  }

  @Log (name="Swerve Module 2")
  public String getSwerveModule2Description() {
    return m_swerveModules[2].getDescription();
  }

  @Log (name="Swerve Module 3")
  public String getSwerveModule3Description() {
    return m_swerveModules[3].getDescription();
  }

  @Log (name="Gyro")
  public String getGyroDescription() {
    // Pitch = hand up, yaw = hand left, roll = hand in.
    String description = "Pitch=" + m_gyro.getPitch() + "    ";
    description += "Yaw=" + m_gyro.getYaw() + "    ";
    description += "Roll=" + m_gyro.getRoll();
    return description;
  }

  public void drive(double leftStickX, double leftStickY, double rightStickX) {
    // Deadband to correct for stick drift.
    leftStickX = deadband(0.25, leftStickX);
    leftStickY = deadband(0.25, leftStickY);
    rightStickX = deadband(0.25, rightStickX);

    // Negatives account for controller stick signs. Note that xVelocity and yVelocity are in robot coordinates.
    double yVelocityMetersPerSecond = -1.0 * leftStickX * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double xVelocityMetersPerSecond = -1.0 * leftStickY * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double rotationVelocityRadiansPerSecond = -1.0 * rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

    // Field oriented to robot speeds.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocityMetersPerSecond,
      yVelocityMetersPerSecond,
      rotationVelocityRadiansPerSecond,
      Rotation2d.fromDegrees(m_gyro.getYaw())
    );

    // Convert to swerve module states.
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    // Pass states to each module.
    for (int location = 0; location < 4; location ++) {
      SwerveModule swerveModule = m_swerveModules[location];
      SwerveModuleState state = states[location];

      swerveModule.setTargetState(state);
    }
  }

  public double deadband(double absDeadbandThreshold, double x) {
    if (Math.abs(x) < absDeadbandThreshold) {
      return 0.0;
    }

    double m = 1.0 / (1 - absDeadbandThreshold);
    return Math.signum(x) * (Math.abs(x) - absDeadbandThreshold) * m;
  }

  @Override
  public void periodic() {}
}
