// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LedController;
import frc.robot.Vision.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralAlignCommand extends Command {
  private final Swerve swerve;
  private final Coral coral;
  private final LedController led;

  private final Timer timer = new Timer();
  private boolean coralDeploymentStarted = false;
  private boolean coralDeployed = false;
  private int coralDeployedCounter = 0;
  private final int coralDeployedThreshold = 3;

  private final PIDController rotationPID;
  private final PIDController distancePID;

  private final int targetLevel;
  private final int targetTagID;

  private double approachSpeed = 0.0;
  private boolean isInApproachPhase = true;

  public CoralAlignCommand(Swerve swerve, Coral coral, LedController led, int targetLevel, int tagID) {
    this.swerve = swerve;
    this.coral = coral;
    this.led = led;
    this.targetLevel = Math.min(Math.max(targetLevel, 1), 4);
    this.targetTagID = tagID;

    switch (targetLevel) {
      case 1:
        rotationPID = new PIDController(0.025, 0.001, 0.001);
        distancePID = new PIDController(0.6, 0, 0.01);
        break;
      case 2:
      case 3:
        rotationPID = new PIDController(0.03, 0.0015, 0.002);
        distancePID = new PIDController(0.5, 0, 0.01);
        break;
      case 4:
        rotationPID = new PIDController(0.035, 0.002, 0.003);
        distancePID = new PIDController(0.4, 0, 0.02);
        break;
      default:
        rotationPID = new PIDController(0.03, 0.001, 0.001);
        distancePID = new PIDController(0.5, 0, 0.01);
    }

    rotationPID.setTolerance(VisionConstants.kAlignmentThreshold);
    double distTolerance = (targetLevel == 1) ? 0.1 : 0.05;
    distancePID.setTolerance(distTolerance);

    addRequirements(swerve, coral, led);
  }

  public CoralAlignCommand(Swerve swerve, Coral coral, LedController led) {
    this(swerve, coral, led, 1, -1);
  }

  @Override
  public void initialize() {
    led.ledColorSetter(LedController.BlinkinPattern.SinelonRainbowPalette);
    timer.reset();
    timer.start();
    coralDeploymentStarted = false;
    coralDeployed = false;
    coralDeployedCounter = 0;
    isInApproachPhase = true;
    rotationPID.reset();
    distancePID.reset();
    int pipelineIndex = (targetLevel <= 2) ? 0 : 1;
    LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightName, pipelineIndex);
    SmartDashboard.putNumber("Target Reef Level", targetLevel);
    SmartDashboard.putString("AutoCoralAlign", "Initialized for L" + targetLevel);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(VisionConstants.kLimelightName)) {
      swerve.drive(new Translation2d(0, 0), 0, false, true);
      coral.moveCoral(0);
      SmartDashboard.putString("AutoCoralAlign", "No valid target detected");
      return;
    }

    double tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);
    double ty = LimelightHelpers.getTY(VisionConstants.kLimelightName);
    double ta = LimelightHelpers.getTA(VisionConstants.kLimelightName);
    double currentTagID = LimelightHelpers.getFiducialID(VisionConstants.kLimelightName);

    if (targetTagID >= 0 && currentTagID != targetTagID) {
      swerve.drive(new Translation2d(0, 0), 0, false, true);
      SmartDashboard.putString("AutoCoralAlign", "Wrong tag ID: " + currentTagID);
      return;
    }

    SmartDashboard.putNumber("AutoCoralAlign/tx", tx);
    SmartDashboard.putNumber("AutoCoralAlign/ty", ty);
    SmartDashboard.putNumber("AutoCoralAlign/ta", ta);
    SmartDashboard.putNumber("AutoCoralAlign/Timer", timer.get());

    double distance;
    double[] botpose = LimelightHelpers.getBotPose_wpiBlue(VisionConstants.kLimelightName);
    if (botpose.length >= 3 && !Double.isNaN(botpose[2])) {
      distance = botpose[2];
    } else {
      double targetHeightMeters = getTargetHeightMeters();
      distance = (targetHeightMeters - VisionConstants.kCameraHeight) /
          Math.tan(Math.toRadians(ty + VisionConstants.kCameraPitchDegrees));
    }

    SmartDashboard.putNumber("AutoCoralAlign/distance", distance);

    double targetDistance = getTargetDistanceForLevel();

    double rotationOutput = rotationPID.calculate(tx, 0);
    double distanceOutput = distancePID.calculate(distance, targetDistance);

    rotationOutput = Math.max(-VisionConstants.kMaxRotationSpeed,
        Math.min(VisionConstants.kMaxRotationSpeed, rotationOutput));
    distanceOutput = Math.max(-VisionConstants.kMaxApproachSpeed,
        Math.min(VisionConstants.kMaxApproachSpeed, distanceOutput));

    if (Math.abs(tx) > VisionConstants.kAlignmentThreshold * 2) {
      distanceOutput = 0;
    }

    boolean aligned = rotationPID.atSetpoint();
    boolean atDistance = distancePID.atSetpoint();

    if (!aligned || !atDistance) {
      Translation2d moveVector;

      if (isInApproachPhase) {
        moveVector = new Translation2d(distanceOutput, 0);
      } else {
        double yAdjustment = -tx * 0.01;
        moveVector = new Translation2d(distanceOutput, yAdjustment);
      }

      swerve.drive(moveVector, rotationOutput, true, true);

      if (coralDeploymentStarted) {
        coral.moveCoral(0);
        coralDeploymentStarted = false;
        coralDeployedCounter = 0;
      }
    } else {
      if (!coralDeploymentStarted) {
        coralDeploymentStarted = true;
        timer.reset();
      }

      swerve.drive(new Translation2d(0, 0), 0, false, true);

      double coralSpeed = getCoralSpeedForLevel();
      coral.moveCoral(coralSpeed);

      if (timer.get() >= getCoralDeployTimeForLevel()) {
        coralDeployedCounter++;
        if (coralDeployedCounter >= coralDeployedThreshold) {
          coralDeployed = true;
        }
      } else {
        coralDeployedCounter = 0;
      }
    }
  }

  private double getTargetDistanceForLevel() {
    switch (targetLevel) {
      case 1:
        return VisionConstants.kL1TargetDistance;
      case 2:
        return VisionConstants.kL2TargetDistance;
      case 3:
        return VisionConstants.kL3TargetDistance;
      case 4:
        return VisionConstants.kL4TargetDistance;
      default:
        return VisionConstants.kL2TargetDistance;
    }
  }

  private double getCoralSpeedForLevel() {
    switch (targetLevel) {
      case 1:
        return VisionConstants.kCoralOutSpeed * 0.8;
      case 2:
        return VisionConstants.kCoralOutSpeed;
      case 3:
        return VisionConstants.kCoralOutSpeed * 1.1;
      case 4:
        return VisionConstants.kCoralOutSpeed * 1.2;
      default:
        return VisionConstants.kCoralOutSpeed;
    }
  }

  private double getCoralDeployTimeForLevel() {
    switch (targetLevel) {
      case 1:
        return VisionConstants.kCoralEjectDuration * 0.8;
      case 2:
        return VisionConstants.kCoralEjectDuration;
      case 3:
        return VisionConstants.kCoralEjectDuration * 1.2;
      case 4:
        return VisionConstants.kCoralEjectDuration * 1.5;
      default:
        return VisionConstants.kCoralEjectDuration;
    }
  }

  private double getTargetHeightMeters() {
    switch (targetLevel) {
      case 1:
        return VisionConstants.kL1HeightMeters;
      case 2:
        return VisionConstants.kL2HeightMeters;
      case 3:
        return VisionConstants.kL3HeightMeters;
      case 4:
        return VisionConstants.kL4HeightMeters;
      default:
        return VisionConstants.kL2HeightMeters;
    }
  }

  @Override
  public boolean isFinished() {
    return coralDeployed;
  }

  @Override
  public void end(boolean interrupted) {
    coral.moveCoral(0);
    led.DefualtColor();
    swerve.drive(new Translation2d(0, 0), 0, false, true);
    timer.stop();
    String endStatus = interrupted ? "Interrupted" : "Completed";
    SmartDashboard.putString("AutoCoralAlign", endStatus + " for L" + targetLevel);
  }
}
