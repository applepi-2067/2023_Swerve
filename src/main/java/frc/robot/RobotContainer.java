// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.PickupPiece;
import frc.robot.commands.SetArmShoulderPosition;
import frc.robot.commands.claw.SpitPiece;
import frc.robot.constants.ArmShoulderPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shoulder;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems.
  private final Drivetrain m_drivetrain;
  private final Shoulder m_shoulder;
  private final Arm m_arm;
  private final Claw m_claw;

  // Controllers.
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;

  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private final CommandXboxController m_operatorController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Create subsystems.
    m_drivetrain = Drivetrain.getInstance();
    m_shoulder = Shoulder.getInstance();
    m_arm = Arm.getInstance();
    m_claw = Claw.getInstance();

    // Create controllers.
    m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Configure the trigger bindings
    configureBindings();

    // Configure logs.
    Logger.configureLoggingAndConfig(this, false);
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
    // Default swerve drive.
    m_drivetrain.setDefaultCommand(
      Commands.run(
        () -> m_drivetrain.drive(
          m_driverController.getLeftX(),
          m_driverController.getLeftY(),
          m_driverController.getRightX()
        ),
        m_drivetrain
      )
    );

    m_driverController.a().onTrue(
      new InstantCommand(
        () -> m_drivetrain.resetGyro()
      )
    );

    // Operator.
    m_operatorController.x().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.STOW));
    m_operatorController.b().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.LOW));

    m_operatorController.povDown().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.Cube.MID));
    m_operatorController.povUp().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.Cube.HIGH));

    m_operatorController.a().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.Cone.MID));
    m_operatorController.y().onTrue(new SetArmShoulderPosition(ArmShoulderPositions.Cone.HIGH));

    m_operatorController.leftTrigger().onTrue(new SpitPiece());

    m_operatorController.povLeft().onTrue(new PickupPiece(ArmShoulderPositions.Pickup.GROUND));
    m_operatorController.rightBumper().onTrue(new PickupPiece(ArmShoulderPositions.Pickup.DOUBLE_SUBSTATION, true));
    m_operatorController.rightTrigger().onTrue(new PickupPiece(ArmShoulderPositions.Pickup.SINGLE_SUBSTATION));
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
