// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoLimitSwitchElevatorMoveToLevelXCommand extends Command {
  /** Creates a new ElevatorMoveToLevelXCommand. */
  private Elevator m_Elevator;
  private int m_RequestedLevel;
  private LedController m_LedController;
  private PIDController m_pidController;
  private double m_levelEncoderSetpoint;

  private final String dbString = "DB/String 4";

  public NoLimitSwitchElevatorMoveToLevelXCommand(Elevator elevator, int requestedLevel, LedController ledController) {
    m_Elevator = elevator;
    m_LedController = ledController;
    m_RequestedLevel = requestedLevel;
    m_pidController = new PIDController(0.02, 1, 0);

    m_levelEncoderSetpoint = m_Elevator.getEncValue(m_RequestedLevel);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator, m_LedController);
  }

  public NoLimitSwitchElevatorMoveToLevelXCommand(Elevator elevator, double encoderValue, LedController ledController) {
    m_Elevator = elevator;
    m_LedController = ledController;
    m_RequestedLevel = 0;
    m_pidController = new PIDController(0.02, 1, 2);

    m_levelEncoderSetpoint = encoderValue;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator, m_LedController);
  }

  private double calculateSpeed() {
    return MathUtil.clamp(
        m_pidController.calculate(m_Elevator.encoderValue()), ElevatorConstant.kElevatorDownSpeed,
        ElevatorConstant.kElevatorMaxMoveSpeed) * m_Elevator.Direction();
  }

  @Override
  public void initialize() {
    m_LedController.setLedColor(BlinkinPattern.RanbowRainbowPalette);
    // m_Elevator.moveElevator(m_Elevator.getStallSpeed());
    // startingMoveDirecrtion = m_Elevator.getMoveDirection(m_RequestedLevel);

    m_pidController.reset();
    m_pidController.setTolerance(2);

    m_pidController.setIntegratorRange(-0.02, 0.02);
    m_pidController.setIZone(m_levelEncoderSetpoint);

    m_pidController.setSetpoint(m_levelEncoderSetpoint);

    SmartDashboard.putString(dbString, "init level " + m_RequestedLevel);

    // m_Elevator.moveElevator(calculateSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DB/String 0", calculateSpeed() + "");
    m_Elevator.moveElevator(calculateSpeed());

    SmartDashboard.putString(dbString, "moving to level " + m_RequestedLevel);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!interrupted && !m_Elevator.getLimitSwitch()) {
      m_LedController.setLedColor(BlinkinPattern.HotPink);

    }
    if (m_Elevator.ElevatorBottomMagnetSwitchState()) {
      m_Elevator.resetEncoderValue();
      m_LedController.setLedColor(BlinkinPattern.Green);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // new StopElevetor(m_LedController);
    return m_pidController.atSetpoint() || m_Elevator.ElevatorBottomMagnetSwitchState();
    // return m_Elevator.getMoveDirection(m_RequestedLevel) !=
    // startingMoveDirecrtion;
    // return false;
  }

  public void setLevel(int level) {
    m_RequestedLevel = level;
    m_levelEncoderSetpoint = m_Elevator.getEncValue(m_RequestedLevel);
  }
}
