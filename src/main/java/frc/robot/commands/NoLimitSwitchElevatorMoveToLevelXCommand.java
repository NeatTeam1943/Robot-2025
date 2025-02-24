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
public class NoLimitSwitchElevatorMoveToLevelXCommand extends Command {
  /** Creates a new ElevatorMoveToLevelXCommand. */
  private Elevator m_Elevator;
  private int m_RequestedLevel;
  private LedController m_LedController;

  public NoLimitSwitchElevatorMoveToLevelXCommand(Elevator elevator, int requestedLevel, LedController ledController) {
    m_Elevator = elevator;
    m_LedController = ledController;
    m_RequestedLevel = requestedLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator, m_LedController);
  }

  @Override
  public void initialize() {
    m_LedController.ledColorSetter(BlinkinPattern.RanbowRainbowPalette);
    m_Elevator.moveElevator(m_Elevator.getStallSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int Direction = m_Elevator.getMoveDiraction(m_RequestedLevel);
    if (Direction != 0) {
      m_Elevator.moveElevator(-(Constants.ElevatorConstants.kElevatorMoveSpeed * Direction));
    } else {
      m_Elevator.moveElevator(m_Elevator.getStallSpeed());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.moveElevator(m_Elevator.getStallSpeed());
    m_LedController.ledColorSetter(BlinkinPattern.HotPink);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.getMoveDiraction(m_RequestedLevel) == 0;
  }
}
