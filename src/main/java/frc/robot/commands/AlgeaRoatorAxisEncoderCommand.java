// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algea;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgeaRoatorAxisEncoderCommand extends Command {
  /** Creates a new AlgeaRoatorAxisEncoder. */
  Algea m_AlgeaRotatorAxis;
  int m_RequestedLevel;

  public AlgeaRoatorAxisEncoderCommand(Algea algeaRotatorAxis, int RequestedLevel) {
    m_AlgeaRotatorAxis = algeaRotatorAxis;
    m_RequestedLevel = RequestedLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_AlgeaRotatorAxis);
  }

  int startingMoveDirection;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingMoveDirection = m_AlgeaRotatorAxis.getMoveDirection(m_RequestedLevel);
    m_AlgeaRotatorAxis.MoveAlgeaAxis(m_AlgeaRotatorAxis.GetStallSpeed());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgeaRotatorAxis.MoveAlgeaAxis(
        Constants.AlgeaConstans.kOpenSpeed * m_AlgeaRotatorAxis.getMoveDirection(m_RequestedLevel));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgeaRotatorAxis.MoveAlgeaAxis(m_AlgeaRotatorAxis.GetStallSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_AlgeaRotatorAxis.getMoveDirection(m_RequestedLevel) != startingMoveDirection;
  }
}
