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
public class ResetTroughBoreCommand extends Command {
  /** Creates a new ResetTroughBoreCommand. */
  private Elevator m_Elevator;
  private LedController m_Leds;

  public ResetTroughBoreCommand(Elevator elevator, LedController leds) {
    m_Elevator = elevator;
    m_Leds = leds;
    addRequirements(m_Elevator, m_Leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.moveElevator(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.moveElevator(Constants.ElevatorConstants.kElevatorDownSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.resetEncoderValue();
    m_Leds.setLedColor(BlinkinPattern.Violet);
    m_Elevator.moveElevator(m_Elevator.getStallSpeed());
    m_Leds.setToDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.ElevatorBottomMagnetSwitchState();
  }
}
