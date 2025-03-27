// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetElevatorCommand extends Command {
  /** Creates a new ElevatorResetCommand. */
  private Elevator m_Elevator;
  private LedController m_LedController;


  public ResetElevatorCommand(Elevator elevator, LedController ledController) {
    m_LedController = ledController;
    m_Elevator = elevator;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  double LastEncoderValue;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LastEncoderValue = m_Elevator.encoderValue();
    m_LedController.setLedColor(BlinkinPattern.RanbowRainbowPalette);
  }

  boolean IsGoingDown;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IsGoingDown = Math.abs(m_Elevator.encoderValue()) - Math.abs(LastEncoderValue ) > 0;
    m_Elevator.moveElevator(Constants.ElevatorConstants.kElevatorDownSpeed * m_Elevator.Direction());
    LastEncoderValue = m_Elevator.encoderValue();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.moveElevator(m_Elevator.getStallSpeed() * m_Elevator.Direction());
    if (m_Elevator.ElevatorBottomMagnetSwitchState()) {
      m_LedController.setLedColor(BlinkinPattern.Green);
      m_Elevator.resetEncoderValue();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.ElevatorBottomMagnetSwitchState()  || (IsGoingDown && m_Elevator.encoderValue() > 1);
  }
}
