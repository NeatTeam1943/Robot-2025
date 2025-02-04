// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorResetCommand extends Command {
  /** Creates a new ElevatorResetCommand. */
  private Elevator m_Elevator;
  private LedController m_LedController;
  public ElevatorResetCommand(Elevator elevator , LedController ledController) {
    m_LedController = ledController;
    m_Elevator = elevator;
    addRequirements(m_Elevator , m_LedController);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedController.LedColorSetter(Constants.LedConstants.kMovingElevatorColor);
    m_Elevator.MoveElevator(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.MoveElevator(-0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      m_LedController.LedColorSetter(Constants.LedConstants.kAtWantedLevelColor);
    }
    m_Elevator.MoveElevator(0);
    m_Elevator.elevatorLevelSetter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.ElevatorBottomLimitState();
  }
}
