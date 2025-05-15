// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IsValidTagCommand extends Command {
  /** Creates a new IsValidTagCommand. */

  public IsValidTagCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("We Started looking");
    SmartDashboard.putNumber("DB/String 4",
        NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("ta").getDouble(0));
    System.out.println(NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("ta").getDouble(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DB/String 3",
        "" + NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("tv").getDouble(0));
    boolean isTarget = NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("tv").getDouble(0) == 1
        ? true
        : false;
    SmartDashboard.putBoolean("DB/LED 2", isTarget);
    SmartDashboard.putNumber("DB/Slider 0",
        NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("ta").getDouble(0));
    System.out.println(NetworkTableInstance.getDefault().getTable("limelight-neat").getEntry("ta").getDouble(0));
    SmartDashboard.putString("DB/String 6", "WE LOOKING AND NOT FINDING SHIT");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DB/String 6", "WE DONE LOOKING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
