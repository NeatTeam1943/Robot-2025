// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeetForwardTestCommand extends Command {
  /** Creates a new FeetForwardTestCommand. */
  Swerve m_Swerve;

  public FeetForwardTestCommand(Swerve swerve) {
    m_Swerve = swerve;
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.drive(new Translation2d(0, 0), 0, false, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Swerve.drive(new Translation2d(SmartDashboard.getNumber("DB/Slider 0", 0) / 1000, 0), 0, false,
        true);
    SmartDashboard.putString("DB/String 5", " " + SmartDashboard.getNumber("DB/Slider 0", 0)/1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
