// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignInFrontOfTag extends Command {
  private Swerve m_Swerve;
  private PIDController m_PidController;
  private PIDController m_RotationPidController;

  /** Creates a new AlignInFrontOfTag. */
  public AlignInFrontOfTag(Swerve swerve) {
    m_Swerve = swerve;
    m_PidController = new PIDController(0, 0, 0);
    m_RotationPidController = new PIDController(0, 0, 0);
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private double calculateHigherSpeed(double Length) {
    return MathUtil.clamp(m_PidController.calculate(Length), Constants.VisionConstants.minSpeed,
        Constants.VisionConstants.maxSpeed);

  }

  private double calculateRotation(double Xangle) {
    return -MathUtil.clamp(m_PidController.calculate(Xangle), Constants.VisionConstants.MaxRotaionSpeed,
        Constants.VisionConstants.MinRotaionSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("DB/LED 1", true);
    m_PidController.reset();
    m_PidController.setTolerance(Constants.VisionConstants.kDistanceErrorTolerence);
    m_PidController.setIntegratorRange(0, 0);
    m_PidController.setIZone(0);
    m_PidController.setSetpoint(Constants.VisionConstants.kWantedDistance);

    m_RotationPidController.reset();
    m_RotationPidController.setTolerance(Constants.VisionConstants.kRoationErrorTolernece);
    m_RotationPidController.setIntegratorRange(0, 0);
    m_RotationPidController.setIZone(0);
    m_RotationPidController.setSetpoint(Constants.VisionConstants.kWantedRotation);
  }

  double rotation;
  double XDistance;
  double YDistance;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isValidTarget = LimelightHelpers.getTV(Constants.VisionConstants.limelightName);
    SmartDashboard.putBoolean("DB/LED 2", isValidTarget);
    XDistance = (Constants.VisionConstants.defaultTargetHeightMeters
        - Constants.VisionConstants.cameraHeightMeters) / Math
            .tan(Math.toRadians(LimelightHelpers.getTY(Constants.VisionConstants.limelightName)));
    YDistance = XDistance
        * Math.tan(Math.toRadians(LimelightHelpers.getTX(Constants.VisionConstants.limelightName))
            - m_Swerve.getHeading().getRadians());
    SmartDashboard.putString("DB/String 0", "" + XDistance);
    SmartDashboard.putString("DB/String 4", "" + YDistance);
    SmartDashboard.putString("DB/String 5", "" + Math.sqrt(Math.pow(XDistance, 2) + Math.pow(YDistance, 2)));

    double xSpeed, ySpeed;
    SmartDashboard.putBoolean("DB/LED 3",
        m_PidController.atSetpoint() || Math.abs(Math.sqrt(Math.pow(XDistance, 2) + Math.pow(YDistance, 2)))
            - Constants.VisionConstants.kDistanceErrorTolerence < Constants.VisionConstants.kWantedDistance);
    if (!m_PidController.atSetpoint() || !(Math.abs(Math.sqrt(Math.pow(XDistance, 2) + Math.pow(YDistance, 2)))
        - Constants.VisionConstants.kDistanceErrorTolerence < Constants.VisionConstants.kWantedDistance)) {
      if (XDistance > YDistance) {
        // xSpeed = calculateHigherSpeed(XDistance);
        xSpeed = Constants.VisionConstants.maxSpeed;
        ySpeed = xSpeed * YDistance / XDistance;
      } else {
        // ySpeed = calculateHigherSpeed(YDistance);
        ySpeed = Constants.VisionConstants.maxSpeed;
        xSpeed = ySpeed * (XDistance / YDistance);
      }
    } else {

      xSpeed = 0;
      ySpeed = 0;
    }
    if (isValidTarget) {
      double rotationSpeed = (Math.abs(LimelightHelpers.getTX(Constants.VisionConstants.limelightName))
          - Constants.VisionConstants.kRoationErrorTolernece) < Constants.VisionConstants.kWantedRotation ? 0
              : calculateRotation(LimelightHelpers.getTX(Constants.VisionConstants.limelightName))
                  * Math.abs(LimelightHelpers.getTX(Constants.VisionConstants.limelightName))
                  / LimelightHelpers.getTX(Constants.VisionConstants.limelightName);
      SmartDashboard.putNumber("DB/Slider 0", ySpeed);
      SmartDashboard.putNumber("DB/Slider 1", xSpeed);
      SmartDashboard.putNumber("DB/Slider 2", rotationSpeed);
      // m_Swerve.runPureVelocity(new ChassisSpeeds(ySpeed , xSpee d, 0));
      m_Swerve.drive(new Translation2d(-xSpeed, ySpeed), rotationSpeed, false, true);
    } else {
      m_Swerve.drive(new Translation2d(0, 0), 0.3, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new Translation2d(0, 0), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PidController.atSetpoint() && m_RotationPidController.atSetpoint())
        || ((Math.abs(LimelightHelpers.getTX(Constants.VisionConstants.limelightName))
            - Constants.VisionConstants.kRoationErrorTolernece) < Constants.VisionConstants.kWantedRotation
            && (Math.abs(Math.sqrt(Math.pow(XDistance, 2) + Math.pow(YDistance, 2)))
                - Constants.VisionConstants.kDistanceErrorTolerence < Constants.VisionConstants.kWantedDistance));
  }
}
