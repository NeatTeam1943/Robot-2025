// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;
import frc.robot.Constants;
import frc.robot.subsystems.CoralOutTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutTakeCommand extends Command {
  /** Creates a new CoralOutTakeCommand. */
  private CoralOutTake m_OutTake;
  private double m_speed;
  private LedController m_LedController;

  public CoralOutTakeCommand(CoralOutTake outTake, double speed, LedController ledController) {
    m_LedController = ledController;
    m_OutTake = outTake;
    m_speed = speed;
    addRequirements(m_OutTake, m_LedController);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_OutTake.moveCoral(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_OutTake.moveCoral(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_OutTake.moveCoral(0);
    if (!m_OutTake.PhotoSwitchMode() && m_speed == Constants.CoralOutTakeConstants.kCoralOutSpeed) {
      m_LedController.DefualtColor();
    } else if (m_OutTake.PhotoSwitchMode() && m_speed == Constants.CoralOutTakeConstants.kCoralInSpeed) {
      m_LedController.LedColorSetter(BlinkinPattern.White);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_speed == Constants.CoralOutTakeConstants.kCoralInSpeed) {
      return m_OutTake.PhotoSwitchMode();
    } else if (m_speed == Constants.CoralOutTakeConstants.kCoralOutSpeed) {
      return !m_OutTake.PhotoSwitchMode();
    } else {
      return false;
    }
  }
}
