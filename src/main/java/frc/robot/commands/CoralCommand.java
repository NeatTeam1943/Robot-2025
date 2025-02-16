// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;
import frc.robot.Constants;

import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralCommand extends Command {
  /** Creates a new CoralOutTakeCommand. */
  private Coral m_coral;
  private double m_speed;
  private LedController m_LedController;
  private boolean m_isIintake;

  public  CoralCommand(Coral coral, boolean isIntake, LedController ledController) {
    m_LedController = ledController;
    m_coral = coral;
    m_isIintake = isIntake;
    m_speed = m_isIintake ? Constants.CoralConstants.kCoralInSpeed : Constants.CoralConstants.kCoralOutSpeed;
    addRequirements(m_coral, m_LedController);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.moveCoral(0);
    System.out.println(m_speed + " = m _ speed");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coral.moveCoral(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(m_coral.PhotoSwitchMode());
    m_coral.moveCoral(0);
    if (!m_isIintake && !m_coral.PhotoSwitchMode()) {
      m_LedController.DefualtColor();
    } else if (m_isIintake && m_coral.PhotoSwitchMode()) {
      m_LedController.LedColorSetter(BlinkinPattern.White);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_isIintake) {
      return m_coral.PhotoSwitchMode();
    } else if (m_isIintake) {
      return !m_coral.PhotoSwitchMode();
    } else {
      return false;
    }
  }
}
