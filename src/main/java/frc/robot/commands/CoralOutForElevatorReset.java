// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LedController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutForElevatorReset extends Command {
  /** Creates a new CoralOutForElevatorReset. */
  private Coral m_Coral;
  private LedController m_LedController;

  public CoralOutForElevatorReset(Coral coral, LedController ledController) {
    m_Coral = coral;
    m_LedController = ledController;
    addRequirements(m_Coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Coral.moveCoral(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Coral.moveCoral(Constants.CoralConstants.kCoralOutSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_LedController.setToDefaultColor();
    }
    m_Coral.moveCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_Coral.PhotoSwitchMode();
  }
}
