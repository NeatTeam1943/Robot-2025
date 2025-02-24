// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveToLevelXLimitSwitch extends Command {
  /** Creates a new ElevatorMoveToLevelXCommand. */
  private Elevator m_Elevator;
  private int m_RequestedLevel;
  private LedController m_LedController;

  public ElevatorMoveToLevelXLimitSwitch(Elevator elevator, int requestedLevel, LedController ledController) {
    m_Elevator = elevator;
    m_LedController = ledController;
    m_RequestedLevel = requestedLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator, m_LedController);
  }

  // Called when the command is initially scheduled.
  private boolean m_LastMagnetSwitchState;
  private boolean m_StartingBottomLimitSwithState;
  private boolean m_StaringTopLimitSwitchState;
  private int m_LevelsToMove;

  @Override
  public void initialize() {
    m_LedController.ledColorSetter(BlinkinPattern.RanbowRainbowPalette);
    m_Elevator.moveElevator(Constants.ElevatorConstants.kStallSpeed);
    m_StartingBottomLimitSwithState = m_Elevator.elevatorBottomLimitState();
    m_StaringTopLimitSwitchState = m_Elevator.elevatorTopLimitState();
    m_LastMagnetSwitchState = m_Elevator.magnetSwitchState();

    m_LevelsToMove = m_RequestedLevel - m_Elevator.getElevatorLevel();
    System.out.println(m_LevelsToMove + "levels to move");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_LevelsToMove);
    if (m_LevelsToMove != 0) {
      m_Elevator
          .moveElevator(m_LevelsToMove / Math.abs(m_LevelsToMove) * Constants.ElevatorConstants.kElevatorMoveSpeed);
      if (m_Elevator.magnetSwitchState() && !m_LastMagnetSwitchState) {
        m_LevelsToMove += -1 * (m_LevelsToMove / Math.abs(m_LevelsToMove));
        m_LastMagnetSwitchState = true;
      } else if (!m_Elevator.magnetSwitchState()) {
        m_LastMagnetSwitchState = false;
      }

    } else
      System.out.println("HELL NAH");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end, interpted: " + interrupted);
    m_Elevator.moveElevator(Constants.ElevatorConstants.kStallSpeed);
    if (!interrupted) {
      if (m_LevelsToMove == 0) {
        m_Elevator.elevatorLevelSetter(m_RequestedLevel);
      }
      m_LedController.ledColorSetter(BlinkinPattern.HotPink);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_StartingBottomLimitSwithState) {
      return m_LevelsToMove == 0 || m_Elevator.elevatorTopLimitState();
    } else if (m_StaringTopLimitSwitchState) {
      return m_LevelsToMove == 0 || m_Elevator.elevatorBottomLimitState();
    } else
      return m_LevelsToMove == 0 || m_Elevator.elevatorBottomLimitState()
          || m_Elevator.elevatorTopLimitState();
  }
}
