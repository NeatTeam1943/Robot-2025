// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.Conversions;
import frc.robot.subsystems.LedController.BlinkinPattern;

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
    m_robotContainer.m_Elevator.resetEncoderValue();

    System.out.println(Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
        Constants.Swerve.kWheelCircumference));

    // SmartDashboard.setDefaultString("DB/String 1",Constants.ElevatorConstants.kL1EncoderValue + "");
    // SmartDashboard.setDefaultString("DB/String 2",Constants.ElevatorConstants.kL2EncoderValue + "");
    // SmartDashboard.setDefaultString("DB/String 3",Constants.ElevatorConstants.kL3EncoderValue + "");
    // SmartDashboard.setDefaultString("DB/String 4",Constants.ElevatorConstants.kL4EncoderValue + "");
    // SmartDashboard.setDefaultString("DB/String 0", "0");
    // SmartDashboard.setDefaultString("DB/String 0", "4.5");
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
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("DB/LED 0", m_robotContainer.getAlgeaSwtich());
    SmartDashboard.putString("DB/String 9", m_robotContainer.getThroBore());
    SmartDashboard.putString("DB/String 8", m_robotContainer.m_Elevator.getStallSpeed() + "");

    // SmartDashboard.putString("DB/String 5", m_robotContainer.)

    // Constants.Swerve.kMaxSpeed =
    // double.parseDouble(SmartDashboard.getString("DB/String 0", "4.5"));
    // SmartDashboard.getString("Max Speed",
    // String.valueOf(Constants.Swerve.kMaxSpeed));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_LedController.ledColorSetter(BlinkinPattern.Orange);
  }

  @Override
  public void disabledPeriodic() {
    switch (m_robotContainer.autoChooser.getSelected()) {
      case "autoChooserTesting":

        SmartDashboard.putData("Auto Chooser1", m_robotContainer.autoChooserTesting);
        break;

      default:
      case "autoChooserGame":
        SmartDashboard.putData("Auto Chooser1", m_robotContainer.autoChooserGame);
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
    m_robotContainer.m_Elevator.resetEncoderValue();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.m_Elevator.resetEncoderValue();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Constants.Swerve.kMaxSpeed =
    // Double.parseDouble(SmartDashboard.getString("DB/String 1", "4.5"));
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