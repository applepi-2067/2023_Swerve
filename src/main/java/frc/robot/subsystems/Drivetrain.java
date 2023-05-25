// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  // TODO: set max speeds.
  // Max speeds.
  private static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 5.0;
  private static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = 5.0;
  
  // Swerve modules.
  private SwerveDriveKinematics m_kinematics;
  private SwerveModule[] m_swerveModules;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    // // Mini swerve bot kinematics.
    // m_kinematics = new SwerveDriveKinematics(
    //   Constants.SwerveModules.MINI.CENTER_OFFSETS[0], Constants.SwerveModules.MINI.CENTER_OFFSETS[1],
    //   Constants.SwerveModules.MINI.CENTER_OFFSETS[2], Constants.SwerveModules.MINI.CENTER_OFFSETS[3]
    // );

    // Go cart kinematics.
    m_kinematics = new SwerveDriveKinematics(
      Constants.SwerveModules.GO_CART.CENTER_OFFSETS[0], Constants.SwerveModules.GO_CART.CENTER_OFFSETS[1],
      Constants.SwerveModules.GO_CART.CENTER_OFFSETS[2], Constants.SwerveModules.GO_CART.CENTER_OFFSETS[3]
    );
    
    m_swerveModules = new SwerveModule[4];
    for (int location = 0; location < 4; location++) {
      m_swerveModules[location] = new SwerveModule(location);
    }
  }

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

  public void drive(double leftStickX, double leftStickY, double rightStickX) {
    // ReLU to correct for stick drift.
    leftStickX = absReLU(0.25, leftStickX);
    leftStickY = absReLU(0.25, leftStickY);
    rightStickX = absReLU(0.25, rightStickX);

    // Negatives account for controller stick signs. Note that xVelocity and yVelocity are in robot coordinates.
    double yVelocityMetersPerSecond = -1.0 * leftStickX * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double xVelocityMetersPerSecond = -1.0 * leftStickY * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double rotationVelocityRadiansPerSecond = -1.0 * rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

    ChassisSpeeds speeds = new ChassisSpeeds(
      xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationVelocityRadiansPerSecond
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

  public double absReLU(double min, double x) {
    if (Math.abs(x) < min) {
      return 0.0;
    }
    return x;
  }
  
  @Override
  public void periodic() {}
}
