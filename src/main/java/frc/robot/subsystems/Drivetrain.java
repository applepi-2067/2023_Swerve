// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.steer.TalonSRXSteerMotor;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = null;

  // // Swerve module center offsets.
  // private static final Translation2d BOTTOM_LEFT_MODULE_CENTER_OFFSET = new Translation2d(-0.5, -0.5);      // TODO: find actual offsets.
  // private static final Translation2d BOTTOM_RIGHT_MODULE_CENTER_OFFSET = new Translation2d(0.5, -0.5);
  // private static final Translation2d FRONT_LEFT_MODULE_CENTER_OFFSET = new Translation2d(-0.5, 0.5);
  // private static final Translation2d FRONT_RIGHT_MODULE_CENTER_OFFSET = new Translation2d(0.5, 0.5);

  // // Swerve module kinematics.
  // private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  //   BOTTOM_LEFT_MODULE_CENTER_OFFSET, BOTTOM_RIGHT_MODULE_CENTER_OFFSET,
  //   FRONT_LEFT_MODULE_CENTER_OFFSET, FRONT_RIGHT_MODULE_CENTER_OFFSET
  // );

  // // Max speeds.
  // private static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 5.0;       // TODO: set max speeds.
  // private static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = 1.5;

  // // Swerve module objects.
  // private SwerveModule m_backLeftSwerveModule;
  // private SwerveModule m_backRightSwerveModule;
  // private SwerveModule m_frontLeftSwerveModule;
  // private SwerveModule m_frontRightSwerveModule;

  // private SwerveModule[] m_swerveModules;

  // Steer motor.
  private TalonSRXSteerMotor m_motor;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    // // Swerve module objects.
    // m_backLeftSwerveModule = new SwerveModule(
    //   Constants.CAN_IDs.SwerveModules.BACK_LEFT_DRIVE,
    //   Constants.CAN_IDs.SwerveModules.BACK_LEFT_STEER
    // );
    // m_backRightSwerveModule = new SwerveModule(
    //   Constants.CAN_IDs.SwerveModules.BACK_RIGHT_DRIVE,
    //   Constants.CAN_IDs.SwerveModules.BACK_RIGHT_STEER
    // );
    // m_frontLeftSwerveModule = new SwerveModule(
    //   Constants.CAN_IDs.SwerveModules.FRONT_LEFT_DRIVE,
    //   Constants.CAN_IDs.SwerveModules.FRONT_LEFT_STEER
    // );
    // m_frontRightSwerveModule = new SwerveModule(
    //   Constants.CAN_IDs.SwerveModules.FRONT_RIGHT_DRIVE,
    //   Constants.CAN_IDs.SwerveModules.FRONT_RIGHT_STEER
    // );

    // m_swerveModules = {
    //   m_backLeftSwerveModule, m_backRightSwerveModule,
    //   m_frontLeftSwerveModule, m_frontRightSwerveModule
    // };

    m_motor = new TalonSRXSteerMotor(Constants.CAN_IDs.SwerveModules.FRONT_LEFT_STEER);
  }

  @Override
  public void periodic() {}

  public void setSteerMotorTargetPositionDegrees(double targetPositionDegrees) {
    m_motor.setTargetPositionDegrees(targetPositionDegrees);
  }

  public void setSteerMotorTargetPositionIncrementDegrees(double targetPositionIncrementDegrees) {
    m_motor.setTargetPositionIncrementDegrees(targetPositionIncrementDegrees);
  }

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
}
