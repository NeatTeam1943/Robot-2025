// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.Authenticator.RequestorType;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveXlevelsCommand extends Command {
  /** Creates a new ElevatorMoveXlevesCommand. */
  Elevator m_Elevator;
  int m_RequestedLevel;
  boolean m_LastMagnetSwitchState;
  boolean m_StartingTopLimitSwitchState;
  boolean m_StartingBottomLimitSwitchState;

  public ElevatorMoveXlevelsCommand(Elevator elvetor, int RequestedLevel) {
    m_Elevator = elvetor;
    m_RequestedLevel = RequestedLevel;
    addRequirements(m_Elevator);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  int m_LevelsToMove;

  @Override
  public void initialize() {

    m_StartingTopLimitSwitchState = m_Elevator.ElevatorTopLimitState();
    m_LevelsToMove = m_StartingBottomLimitSwitchState = m_Elevator.ElevatorBottomLimitState();
    m_LastMagnetSwitchState = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int m_ElevatorLevel = m_Elevator.elevatorLevelGetter();
    System.out.println("Robot Level Is : " + m_Elevator.elevatorLevelGetter());
    if (m_ElevatorLevel != 0) {
      m_Elevator.MoveElevator((m_ElevatorLevel / Math.abs(m_ElevatorLevel)) * 0.05);
      if (m_Elevator.MagnetSwitchState() && !m_LastMagnetSwitchState) {
        if (m_RequestedLevel > m_Elevator.ElevatorLevel()) {
          m_Elevator.elevatorLevelSetter(m_ElevatorLevel + 1);
        } else
          m_LastMagnetSwitchState = true;
      } else if (!m_Elevator.MagnetSwitchState()) {
        m_LastMagnetSwitchState = false;
      }
    } else
      System.out.println("we hit a roadblock idky why");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.MoveElevator(0);
    m_Elevator.elevatorLevelSetter(m_RequestedLevel);
    System.out.println("Robot Level Is : " + m_Elevator.elevatorLevelGetter());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_StartingTopLimitSwitchState) {
      return m_Elevator.elevatorLevelGetter() == m_RequestedLevel || m_Elevator.ElevatorBottomLimitState();
    } else if (m_StartingBottomLimitSwitchState) {
      return m_Elevator.elevatorLevelGetter() == m_RequestedLevel || m_Elevator.ElevatorTopLimitState();
    } else
      return m_Elevator.elevatorLevelGetter() == m_RequestedLevel || m_Elevator.ElevatorBottomLimitState()
          || m_Elevator.ElevatorTopLimitState();
  }
}