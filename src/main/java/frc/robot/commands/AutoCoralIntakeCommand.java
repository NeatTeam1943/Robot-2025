// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController.BlinkinPattern;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCoralIntakeCommand extends Command {

  /** Creates a new CoralOutTakeCommand. */
  private Coral m_coral;
  private LedController m_LedController;
  private CommandXboxController m_Controller1;
  private CommandXboxController m_Controller2;
  private boolean m_isThereACoral;

  public AutoCoralIntakeCommand(Coral coral, LedController ledController, CommandXboxController controller1, CommandXboxController controller2) {
    m_LedController = ledController;
    m_coral = coral;
    m_Controller1 = controller1;
    m_Controller2 = controller2;

    addRequirements(m_coral);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isThereACoral = m_coral.IntakePhotoSwitchMode() && !m_coral.PhotoSwitchMode();
    m_coral.moveCoral(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coral.IntakePhotoSwitchMode() && !m_coral.PhotoSwitchMode()) {
      m_coral.moveCoral(Constants.CoralConstants.kCoralInSpeed);
    } else {
      m_coral.moveCoral(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.moveCoral(0);
    if (!interrupted) {
      if (m_isThereACoral) {
        m_LedController.setLedColor(BlinkinPattern.White);
      }
    }
    m_Controller1.setRumble(RumbleType.kBothRumble, 0);
    m_Controller2.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_Controller1.setRumble(RumbleType.kBothRumble, 1);
    m_Controller2.setRumble(RumbleType.kBothRumble, 1);
    return m_coral.PhotoSwitchMode() || !m_coral.IntakePhotoSwitchMode();
  }
}
