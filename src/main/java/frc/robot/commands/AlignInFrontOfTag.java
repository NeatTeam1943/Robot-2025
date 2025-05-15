// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignInFrontOfTag extends Command {
  private Swerve m_Swerve;
  private PIDController m_PidController;

  /** Creates a new AlignInFrontOfTag. */
  public AlignInFrontOfTag(Swerve swerve) {
    m_Swerve = swerve;
    m_PidController = new PIDController(0, 0, 0);
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private double calculateHigherSpeed(double Length) {
    return MathUtil.clamp(m_PidController.calculate(Length), Constants.VisionConstants.minSpeed,
        Constants.VisionConstants.maxSpeed);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PidController.reset();
    m_PidController.setTolerance(0);
    m_PidController.setIntegratorRange(0, 0);
    m_PidController.setIZone(0);
    m_PidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV(Constants.VisionConstants.limelightName)){
      double XLength = Math.tan(Math.toRadians(LimelightHelpers.getTY(Constants.VisionConstants.limelightName))) * ;
    }
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
