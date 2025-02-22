// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorFullExtend extends Command {
  /** Creates a new ElevatorFullExtend. */
  private Elevator m_Elevator;

  public ElevatorFullExtend(Elevator elevator) {
    m_Elevator = elevator;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.moveElevator(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.moveElevator(0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.moveElevator(0);
    m_Elevator.elevatorLevelSetter(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.elevatorTopLimitState();
  }
}
