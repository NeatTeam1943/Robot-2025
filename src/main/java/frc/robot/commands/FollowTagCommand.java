// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.transaction.xa.Xid;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowTagCommand extends Command {
  /** Creates a new FollowTagCommand. */
  Swerve m_Swerve;

  public FollowTagCommand(Swerve swerve) {
    m_Swerve = swerve;
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  ChassisSpeeds cSpeeds;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean isValidTarget = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.limelightName)
        .getEntry("tv").getDouble(0) == 1.0;
    SmartDashboard.putBoolean("DB/LED 2", isValidTarget);
    double XDistance = (Constants.VisionConstants.defaultTargetHeightMeters
        - Constants.VisionConstants.cameraHeightMeters) / Math
            .tan(Math.toRadians(NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.limelightName)
                .getEntry("ty").getDouble(0)));
    double YDistance = XDistance * Math.tan(Math.toRadians(NetworkTableInstance.getDefault()
        .getTable(Constants.VisionConstants.limelightName).getEntry("tx").getDouble(0)));
    SmartDashboard.putNumber("DB/Slider 2", YDistance);
    SmartDashboard.putNumber("DB/Slider 1", XDistance);
    double xSpeed , ySpeed;
    if(XDistance > YDistance){
       xSpeed = Constants.VisionConstants.maxSpeed;
       ySpeed = xSpeed * (YDistance/XDistance);
    }
    else{
      ySpeed = Constants.VisionConstants.maxSpeed;
      xSpeed = ySpeed * (YDistance/XDistance);
    }
    if (isValidTarget) {
      m_Swerve.runPureVelocity(new ChassisSpeeds(ySpeed , xSpeed , 0));
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
