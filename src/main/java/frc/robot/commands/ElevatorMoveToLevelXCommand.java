// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.ClosedWatchServiceException;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveToLevelXCommand extends Command {
  /** Creates a new ElevatorMoveToLevelXCommand. */
  private Elevator m_Elevator;
  private int m_RequestedLevel;

  public ElevatorMoveToLevelXCommand(Elevator elevator, int requestedLevel) {
    m_Elevator = elevator;
    m_RequestedLevel = requestedLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  private boolean m_LastMagnetSwitchState;
  private boolean m_StartingBottomLimitSwithState;
  private boolean m_StaringTopLimitSwitchState;
  private int m_LevelsToMove;

  @Override
  public void initialize() {
    m_Elevator.MoveElevator(0);
    m_StartingBottomLimitSwithState = m_Elevator.ElevatorBottomLimitState();
    m_StaringTopLimitSwitchState = m_Elevator.ElevatorTopLimitState();
    m_LastMagnetSwitchState = m_Elevator.MagnetSwitchState();

    m_LevelsToMove = m_RequestedLevel - m_Elevator.elevatorLevelGetter();
    System.out.println(m_LevelsToMove + "levels to move");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_LevelsToMove);
    if (m_LevelsToMove != 0) {
      m_Elevator.MoveElevator(m_LevelsToMove / Math.abs(m_LevelsToMove) * 0.05);
      if (m_Elevator.MagnetSwitchState() && !m_LastMagnetSwitchState) {
        m_LevelsToMove += -1 * (m_LevelsToMove / Math.abs(m_LevelsToMove));
        m_LastMagnetSwitchState = true;
      } else if (!m_Elevator.MagnetSwitchState()) {
        m_LastMagnetSwitchState = false;
      }
    } else
      System.out.println("oops");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end, interpted: " + interrupted);
    m_Elevator.MoveElevator(0);
    if (!interrupted) {
      m_Elevator.elevatorLevelSetter(m_RequestedLevel);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_StartingBottomLimitSwithState) {
      return m_LevelsToMove == 0 || m_Elevator.ElevatorTopLimitState();
    } else if (m_StaringTopLimitSwitchState) {
      return m_LevelsToMove == 0 || m_Elevator.ElevatorBottomLimitState();
    } else
      return m_LevelsToMove == 0 || m_Elevator.ElevatorBottomLimitState()
          || m_Elevator.ElevatorTopLimitState();
  }
}
