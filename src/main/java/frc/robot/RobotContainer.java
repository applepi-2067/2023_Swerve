// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Create subsystems.
  Drivetrain m_drivetrain = Drivetrain.getInstance();

  // Create controllers.
  private final CommandXboxController m_driverController = 
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // Configure logs.
    Logger.configureLoggingAndConfig(this, false);

    // Configure the trigger bindings
    configureBindings();

    // // Default swerve drive.
    // m_drivetrain.setDefaultCommand(
    //   Commands.run(
    //     () -> m_drivetrain.drive(
    //       m_driverController.getLeftX(),
    //       m_driverController.getLeftY(),
    //       m_driverController.getRightX()
    //     ), m_drivetrain)
    // );

    m_drivetrain.setDefaultCommand(
      Commands.run(
        () -> m_drivetrain.setDriveMotorTargetVelocityMetersPerSecond(m_driverController.getRightY()),
        m_drivetrain
      )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(
      Commands.run(
        () -> m_drivetrain.setSteerMotorTargetPositionDegrees(0),
        m_drivetrain
      )
    );

    m_driverController.b().onTrue(
      Commands.run(
        () -> m_drivetrain.setSteerMotorTargetPositionDegrees(90.0),
        m_drivetrain
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
