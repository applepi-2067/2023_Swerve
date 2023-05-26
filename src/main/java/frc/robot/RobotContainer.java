// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private Drivetrain m_drivetrain;

  // Controllers.
  private CommandXboxController m_driverXBoxController = null;
  private CommandJoystick m_driverJoystickController = null;

  // Go-cart boolean.
  private DigitalInput m_goCartSensor;
  private boolean isGoCart;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Figure out if we are using the go-cart.
    m_goCartSensor = new DigitalInput(Constants.DigitalInputs.GO_CART_SENSOR_DI);
    isGoCart = !m_goCartSensor.get();
    
    // Create drivetrain.
    m_drivetrain = Drivetrain.getInstance(isGoCart);
    SmartDashboard.putBoolean("isGoCart sensor", isGoCart);

    // Create controller.
    if (isGoCart) {
      m_driverJoystickController = new CommandJoystick(OperatorConstants.kDriverJoystickControllerPort);
    }
    else {
      m_driverXBoxController = new CommandXboxController(OperatorConstants.kDriverXBoxControllerPort);
    }

    // Configure the trigger bindings
    configureBindings(isGoCart);

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
  private void configureBindings(boolean isGoCart) {
    if (isGoCart) {
      m_driverJoystickController.setTwistChannel(3);

      // Go cart drive.
      m_drivetrain.setDefaultCommand(
        Commands.run(
          () -> m_drivetrain.drive(
            -1.0 * m_driverJoystickController.getX(),   // TODO: why is this inverted along with center offsets?
            m_driverJoystickController.getY(),
            -1.0 * m_driverJoystickController.getTwist()
          ), m_drivetrain)
      );
    }
    else {
      // Default swerve drive.
      m_drivetrain.setDefaultCommand(
        Commands.run(
          () -> m_drivetrain.drive(
            m_driverXBoxController.getLeftX(),
            m_driverXBoxController.getLeftY(),
            m_driverXBoxController.getRightX()
          ), m_drivetrain)
      );
    }
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
