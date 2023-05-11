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
import frc.robot.subsystems.swerve.drive.SparkMaxDriveMotor;
import frc.robot.subsystems.swerve.steer.TalonSRXSteerMotor;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  // Max speeds.
  private static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 5.0;       // TODO: set max speeds.
  private static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = 1.5;
  
  // Swerve modules.
  // private SwerveDriveKinematics m_kinematics;
  // private SwerveModule[] m_swerveModules;

  // Testing motors.
  private TalonSRXSteerMotor m_steerMotor;
  private SparkMaxDriveMotor m_driveMotor;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    // m_kinematics = new SwerveDriveKinematics(
    //   Constants.SwerveModules.CENTER_OFFSETS[0], Constants.SwerveModules.CENTER_OFFSETS[1],
    //   Constants.SwerveModules.CENTER_OFFSETS[2], Constants.SwerveModules.CENTER_OFFSETS[3]
    // );
    
    // m_swerveModules = new SwerveModule[4];
    // for (int location = 0; location < 4; location++) {
    //   m_swerveModules[location] = new SwerveModule(location);
    // }

    m_steerMotor = new TalonSRXSteerMotor(2);
    m_driveMotor = new SparkMaxDriveMotor(2);
  }

  public void setSteerMotorTargetPositionDegrees(double targetPositionDegrees) {
    m_steerMotor.setTargetPositionDegrees(targetPositionDegrees);
  }

  public void setDriveMotorTargetVelocityMetersPerSecond(double velocityMetersPerSecond) {
    m_driveMotor.setTargetVelocityMetersPerSecond(velocityMetersPerSecond);
  }

  // public void setSwerveModuleState(double steerMotorTargetPositionDegrees, double driveMotorTargetVelocityMetersPerSecond) {
  //   for (int location = 0; location < 4; location++) {
  //     SwerveModuleState state = new SwerveModuleState(
  //       driveMotorTargetVelocityMetersPerSecond, Rotation2d.fromDegrees(steerMotorTargetPositionDegrees)
  //     );
  //     m_swerveModules[location].setTargetState(state);
  //   }
  // }

  // @Log (name="Swerve Module 0")
  // public String getSwerveModule0Description() {
  //   String description = "Location 0:";
  //   description += " ticks=" + m_steerMotors[0].getPositionTicks();
  //   description += " degrees=" + m_steerMotors[0].getPositionDegrees();
  //   return description;
  // }

  // @Log (name="Swerve Module 1")
  // public String getSwerveModule1Description() {
  //   String description = "Location 1:";
  //   description += " ticks=" + m_steerMotors[1].getPositionTicks();
  //   description += " degrees=" + m_steerMotors[1].getPositionDegrees();
  //   return description;
  // }

  // @Log (name="Swerve Module 2")
  // public String getSwerveModule2Description() {
  //   String description = "Location 2:";
  //   description += " ticks=" + m_steerMotors[2].getPositionTicks();
  //   description += " degrees=" + m_steerMotors[2].getPositionDegrees();
  //   return description;
  // }

  // @Log (name="Swerve Module 3")
  // public String getSwerveModule3Description() {
  //   String description = "Location 3:";
  //   description += " ticks=" + m_steerMotors[3].getPositionTicks();
  //   description += " degrees=" + m_steerMotors[3].getPositionDegrees();
  //   return description;
  // }

  // public void drive(double leftStickX, double leftStickY, double rightStickX) {
  //   double xVelocityMetersPerSecond = leftStickX * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
  //   double yVelocityMetersPerSecond = leftStickY * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
  //   double rotationVelocityRadiansPerSecond = rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

  //   ChassisSpeeds speeds = new ChassisSpeeds(
  //     xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationVelocityRadiansPerSecond
  //   );

  //   // Convert to swerve module states.
  //   SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

  //   // Pass states to each module.
  //   for (int location = 0; location < 4; location ++) {
  //     SwerveModule swerveModule = m_swerveModules[location];
  //     SwerveModuleState state = states[location];

  //     swerveModule.setTargetState(state);
  //   }
  // }
  
  @Override
  public void periodic() {}
}
