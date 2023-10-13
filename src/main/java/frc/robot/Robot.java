package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ScoreAndLeaveCommunity;
import frc.robot.commands.ScorePreloadedPiece;
import io.github.oblarg.oblog.Logger;


// VM will automatically call functions in this class.
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  
  public enum AutoChoice {
    SCORE_PRELOADED_PIECE,
    SCORE_AND_LEAVE_COMMUNITY
  }

  private Command m_autoCommand;
  private SendableChooser<AutoChoice> m_autoChooser;

  // Ran when the robot is first booted up.
  @Override
  public void robotInit() {
    // Init robot container.
    m_robotContainer = new RobotContainer();

    // Create auto chooser.
    m_autoChooser = new SendableChooser<AutoChoice>();
    m_autoChooser.setDefaultOption("ScorePreloadedPiece", AutoChoice.SCORE_PRELOADED_PIECE);
    m_autoChooser.addOption("ScoreAndLeaveCommunity", AutoChoice.SCORE_AND_LEAVE_COMMUNITY);
  }

  // Called every 20ms no matter what mode.
  @Override
  public void robotPeriodic() {
    /**
     * Run command scheduler.
     * - Poll buttons
     * - Schedule / run / kill commands
     * - Run subsustem periodic.
     */
    CommandScheduler.getInstance().run();

    // Update logs.
    Logger.updateEntries();
  }

  // Called when robot enters disabled mode.
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Put auto chooser on SmartDashboard.
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  // Run auto command.
  @Override
  public void autonomousInit() {
    AutoChoice autoChoice = m_autoChooser.getSelected();
    if (autoChoice != null) {
      if (autoChoice == AutoChoice.SCORE_PRELOADED_PIECE) {
        m_autoCommand = new ScorePreloadedPiece();
      }
      else if (autoChoice == AutoChoice.SCORE_AND_LEAVE_COMMUNITY) {
        m_autoCommand = new ScoreAndLeaveCommunity();
      }
      m_autoCommand.schedule();
    }
  }

  // Called periodically during auto.
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Kill auto commands and then start teleop.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  // Called periodically in teleop.
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
