// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveEleveatorTestMagnetHeight extends Command {
  /** Creates a new MoveEleveatorTestMagnetHeight. */
  private Elevator m_Elevator;
  private CommandXboxController m_Controller;

  public MoveEleveatorTestMagnetHeight(Elevator elevator, CommandXboxController controller) {
    m_Controller = controller;
    m_Elevator = elevator;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  double m_Speed;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Speed = m_Elevator.getStallSpeed();
    m_Elevator.moveElevator(m_Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Speed = Math.abs(m_Controller.getLeftY()) > 0.1 ? -m_Controller.getLeftY()
        : m_Elevator.getStallSpeed();
    m_Elevator.moveElevator(m_Speed * m_Elevator.Direction());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Interrupted is : "  + interrupted  );
    m_Speed = m_Elevator.getStallSpeed();
    m_Elevator.moveElevator(m_Speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
