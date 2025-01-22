// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveXlevelsCommand extends Command {
  /** Creates a new ElevatorMoveXlevesCommand. */
  Elevator m_Elevator;
  int m_RequestedLevel;
  public ElevatorMoveXlevelsCommand(Elevator elvetor , int RequestedLevel) {
    m_Elevator = elvetor;
    m_RequestedLevel = RequestedLevel;
    addRequirements(m_Elevator);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  int m_LevelsToMove;
  @Override
  public void initialize() {
    m_LevelsToMove = m_RequestedLevel - m_Elevator.ElevatorLevel();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.MoveElevator((m_LevelsToMove/Math.abs(m_LevelsToMove))*0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.MoveElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LevelsToMove == 0 || m_Elevator.ElevatorBottomLimitState() || m_Elevator.ElevatorTopLimitState();
  }
}