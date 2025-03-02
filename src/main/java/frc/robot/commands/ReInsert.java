// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LedController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReInsert extends Command {

  private Coral m_Coral;
  private LedController m_Led;
  
  public ReInsert(Coral coral, LedController led) {
    m_Coral = coral;
    m_Led = led;
    addRequirements(m_Coral, m_Led);
  }
  
  private boolean cycled;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Coral.moveCoral(0);
    cycled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_Coral.PhotoSwitchMode()){
      cycled = true;
    }
    m_Coral.moveCoral(!cycled ? -0.2 : frc.robot.Constants.CoralConstants.kCoralInSpeed);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Coral.moveCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cycled && m_Coral.PhotoSwitchMode();
  }
}
