// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgeaRotatorAxis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgeaRotatorAxisCommand extends Command {

  /** Creates a new AlgeaRotatroAxis. */
  private AlgeaRotatorAxis m_AlgeaRotatorAxis;

  private CommandXboxController m_Controller;

  public AlgeaRotatorAxisCommand(AlgeaRotatorAxis rotatorAxis, CommandXboxController controller) {
    m_AlgeaRotatorAxis = rotatorAxis;
    m_Controller = controller;
    addRequirements(m_AlgeaRotatorAxis);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    m_AlgeaRotatorAxis.AlgeaRotatorAxisMove(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgeaRotatorAxis.AlgeaRotatorAxisMove(
        m_Controller.getRightTriggerAxis() - m_Controller.getLeftTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgeaRotatorAxis.AlgeaRotatorAxisMove(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
