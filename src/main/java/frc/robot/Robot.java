// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static EventLoop m_eventLoop = new EventLoop();

  public static EventLoop getEventLoop() {
    return m_eventLoop;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Rotation2d angle = new Rotation2d(0);
    SwerveModuleState desiredState = new SwerveModuleState(2, angle);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    m_eventLoop.poll();

    CommandScheduler.getInstance().run();
    SmartDashboard.putString("DB/String 9", m_robotContainer.getElevator().encoderValue() + "");
    SmartDashboard.putString("Elevator Ecoder:", m_robotContainer.getElevator().encoderValue() + "");
    SmartDashboard.putString("Elevator magent:", m_robotContainer.getElevator().ElevatorBottomMagnetSwitchState() + "");
    SmartDashboard.putString("DB/String 8", m_robotContainer.getElevator().getStallSpeed() + "");
    SmartDashboard.putString("Gyro Yaw:", m_robotContainer.getSwerve().getGyroYaw().getDegrees() + "");
    m_robotContainer.getCoral().getState();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setToNeatTeamLED();
  }

  @Override
  public void disabledPeriodic() {
    // Don't remove this section this is how you chouse an auto for PP
    switch (m_robotContainer.m_AutoChooser.getSelected()) {
      case "autoChooserTesting":

        SmartDashboard.putData("Auto Chooser1", m_robotContainer.m_AutoChooserTesting);
        break;

      default:
      case "autoChooserGame":
        SmartDashboard.putData("Auto Chooser1", m_robotContainer.m_AutoChooserGame);
        break;
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.getSwerve().getVelocityAll();

    m_robotContainer.m_LedController.setToDefaultColor();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.getSwerve().getVelocityAll();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.resetGyro();
    m_robotContainer.m_LedController.setToDefaultColor();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.getSwerve().getVelocityAll();

    SmartDashboard.putString("DB/String 7", m_robotContainer.getALgea().GetEncoderValue() + "");
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}