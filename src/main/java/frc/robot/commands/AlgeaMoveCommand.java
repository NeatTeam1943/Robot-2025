// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algea;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgeaMoveCommand extends Command {
  /** Creates a new AlgeaInCommand. */
  private Algea m_AlgeaIn;
  private int m_Direction;

  public AlgeaMoveCommand(Algea algea, int Direction) {
    m_AlgeaIn = algea;
    m_Direction = Direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_AlgeaIn);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgeaIn.SetAlgeaSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgeaIn.SetAlgeaSpeed(0.25 * m_Direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgeaIn.SetAlgeaSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Direction == -1) {
      return !m_AlgeaIn.PhotoSwitchState();
    } else {
      return m_AlgeaIn.PhotoSwitchState();
    }
  }
}
