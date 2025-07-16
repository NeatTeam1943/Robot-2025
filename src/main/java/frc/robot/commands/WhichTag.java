// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WhichTag extends Command {
  /** Creates a new WhichTag. */
  public WhichTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DB/String 6",
        LimelightHelpers.getFiducialID(Constants.VisionConstants.limelightName) + "");
    // SmartDashboard.putString("DB/String 0",
    //     "" + LimelightHelpers.getBotPose2d(Constants.VisionConstants.limelightName).getX() + "   "
    //         + LimelightHelpers.getBotPose2d(Constants.VisionConstants.limelightName).getY());
    System.out.println("X value - " + LimelightHelpers.getBotPose2d(Constants.VisionConstants.limelightName).getX());
    System.out.println("Y value - " + LimelightHelpers.getBotPose2d(Constants.VisionConstants.limelightName).getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Returns true when the command should end.
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
