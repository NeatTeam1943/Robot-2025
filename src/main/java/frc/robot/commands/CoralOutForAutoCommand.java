// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutForAutoCommand extends Command {
  /** Creates a new CoralOutForAutoCommand. */
  Coral m_coral;
  public CoralOutForAutoCommand(Coral coral) {
    m_coral = coral;
    addRequirements(m_coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.moveCoral(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coral.moveCoral(Constants.CoralConstants.kCoralOutSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.moveCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_coral.PhotoSwitchMode();
  }
}
