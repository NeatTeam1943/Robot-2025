// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOutTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralTransportOutTake extends Command {
  /** Creates a new CoralTransport(OutTake). */
  CoralOutTake m_OutTransport;
  int m_timer;
  public CoralTransportOutTake(CoralOutTake outTransport) {
    m_OutTransport = outTransport;
    addRequirements(m_OutTransport);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_OutTransport.moveCoral(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer < 25){ // 5 ticks == 0.1 sec
      m_timer++;
    }
    else {
      m_OutTransport.moveCoral(0.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_OutTransport.moveCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_OutTransport.PhotoSwitchMode();
  }
}
