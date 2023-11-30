// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.swerve.SwerveModule;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  // HACK: Why are left and right switched?
  // Swerve module offsets from center.
  // NOTE: +x = front of robot, +y = left of robot.
  private static final double halfWheelBaseMeters = Units.inchesToMeters(9.75);
  private static final Translation2d[] SWERVE_MODULE_CENTER_OFFSETS = {
    new Translation2d(-halfWheelBaseMeters, -halfWheelBaseMeters),
    new Translation2d(-halfWheelBaseMeters, halfWheelBaseMeters),
    new Translation2d(halfWheelBaseMeters, -halfWheelBaseMeters),
    new Translation2d(halfWheelBaseMeters, halfWheelBaseMeters),
  };

  // TODO: Set max velocity.
  // Max speeds.
  private static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 4.57;
  private static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = Math.PI * 4;
  
  // Swerve modules.
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;
  private final SwerveModule[] m_swerveModules;

  private final PigeonIMU m_gyro;

  private Pose2d m_pose;

  private final Field2d m_field;

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
    TalonSRX gyroController = new TalonSRX(RobotMap.canIDs.Drivetrain.GYRO);
    m_gyro = new PigeonIMU(gyroController);
    m_gyro.setYaw(0.0);

    // Odometry.
    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      getSwerveModulePositions(),
      new Pose2d()
    );

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_pose = getRobotPose2d();
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

  @Log (name="Robot Pose")
  public String getPoseDescription() {
    String poseString = "x (m)" + m_pose.getX() + "    ";
    poseString += "y (m)" + m_pose.getY() + "    ";
    poseString += "rotation (deg)" + m_pose.getRotation().getDegrees();
    return poseString;
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      swerveModulePositions[i] = m_swerveModules[i].getPosition();
    }
    return swerveModulePositions;
  }

  public void drive(double leftStickX, double leftStickY, double rightStickX) {
    // Deadband to correct for stick drift.
    double absDeadbandThreshold = 0.05;
    leftStickX = deadband(absDeadbandThreshold, leftStickX);
    leftStickY = deadband(absDeadbandThreshold, leftStickY);
    rightStickX = deadband(absDeadbandThreshold, rightStickX);

    // Square stick inputs.
    leftStickX = Math.pow(leftStickX, 2.0) * Math.signum(leftStickX);
    leftStickY = Math.pow(leftStickY, 2.0) * Math.signum(leftStickY);
    rightStickX = Math.pow(rightStickX, 2.0) * Math.signum(rightStickX);

    // HACK: Why are y and rotation not inverted?
    // Negatives account for controller stick signs. Note that xVelocity and yVelocity are in robot coordinates.
    double yVelocityMetersPerSecond = leftStickX * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double xVelocityMetersPerSecond = -1.0 * leftStickY * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double rotationVelocityRadiansPerSecond = rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

    // BUG: Field oriented causes unoptimized rotation?
    // Field oriented to robot speeds.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocityMetersPerSecond,
      yVelocityMetersPerSecond,
      rotationVelocityRadiansPerSecond,
      Rotation2d.fromDegrees(-1.0 * m_gyro.getYaw())
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

  public void resetGyro() {
    m_gyro.setYaw(0.0);
  }

  public double getGyroPitch() {
    return m_gyro.getPitch();
  }

  public Pose2d getRobotPose2d() {
    Pose2d robotPose2d = m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      getSwerveModulePositions()
    );
    return robotPose2d; 
  }

  @Override
  public void periodic() {
    m_pose = getRobotPose2d();
    m_field.setRobotPose(m_pose);
  }
    AutoBuilder.configureHolonomic(
      getRobotPose2d, // Robot pose supplier
      resetGyro, // Method to reset odometry (will be called if your auto has a starting pose)
      ChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      fromFieldRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    )
  )
}
