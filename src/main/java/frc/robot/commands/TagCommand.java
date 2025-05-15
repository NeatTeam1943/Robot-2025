package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TagCommand extends Command {
    private final Swerve m_swerve;
    private boolean testMode = false;
    private boolean active = true;

    public TagCommand(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public void setTestMode(boolean test) {
        testMode = test;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setCameraMode_Processor(Constants.VisionConstants.limelightName);
        LimelightHelpers.setLEDMode_ForceOn(Constants.VisionConstants.limelightName);
        int pipeline = testMode ? 1 : 0;
        LimelightHelpers.setPipelineIndex(Constants.VisionConstants.limelightName, pipeline);
        configureLimelight();
    }

    @Override
    public void execute() {
        if (!active) {
            m_swerve.drive(new Translation2d(0, 0), 0, false, true);
            SmartDashboard.putString("DB/String 0", "AprilTag Follow inactive");
            return;
        } else {
            SmartDashboard.putString("DB/String 0", "AprilTag Follow Active");
        }
        LimelightResults results = LimelightHelpers.getLatestResults(Constants.VisionConstants.limelightName);

        if (!results.targetingResults.valid || results.targetingResults.targets_Fiducials.length == 0) {
            m_swerve.drive(new Translation2d(0, 0), 0.2, false, true);
            SmartDashboard.putString("DB/String 4", "Searching for tag...");
            return;
        }

        LimelightTarget_Fiducial closestTag = selectClosestTag(results.targetingResults.targets_Fiducials);
        int fiducialID = (int) closestTag.fiducialID;
        double tagHeight = getAprilTagHeight(fiducialID);
        double distance = calculateDistance(closestTag.ty, tagHeight);
        double angleError = Units.degreesToRadians(closestTag.tx);

        Pose2d targetPose = getAprilTagPose(fiducialID);
        if (targetPose == null) {
            // DriverStation.reportWarning("Invalid tag ID: " + fiducialID, false);
            SmartDashboard.putString("DB/String 3", "TargetPose = null");
            m_swerve.drive(new Translation2d(0, 0), 0, false, true);
            return;
        }

        // if (!isStrategicAprilTag(fiducialID) && distance > Constants.VisionConstants.maxAprilTagDistance) {
        //     m_swerve.drive(new Translation2d(0, 0), 0, false, true);
        //     DriverStation.reportWarning("Non-strategic tag too far: " + fiducialID, false);
        //     return;
        // }

        double linearSpeed = Math.min(distance * 0.3, Constants.SwerveConstans.kMaxSpeed);
        if (distance < Constants.VisionConstants.slowdownDistance) {
            linearSpeed *= (distance / Constants.VisionConstants.slowdownDistance);
        }
        double angularSpeed = Math.copySign(
                Math.min(Math.abs(angleError * 1.5), Constants.SwerveConstans.kMaxAngularVelocity),
                angleError);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearSpeed * Math.cos(angleError),
                linearSpeed * Math.sin(angleError),
                angularSpeed,
                m_swerve.getHeading());

        if (willCollideWithFieldElement(chassisSpeeds, targetPose)) {
            DriverStation.reportWarning("Collision risk detected.", false);
            m_swerve.drive(new Translation2d(0, 0), 0, false, true);
            return;
        }

        m_swerve.drive(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                chassisSpeeds.omegaRadiansPerSecond,
                true, false);

        Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.VisionConstants.limelightName);
        if (visionPose != null) {
            Pose2d currentPose = m_swerve.getPose();
            double visionWeight = 0.3;
            double odometryWeight = 1.0 - visionWeight;
            Pose2d blendedPose = new Pose2d(
                    currentPose.getX() * odometryWeight + visionPose.getX() * visionWeight,
                    currentPose.getY() * odometryWeight + visionPose.getY() * visionWeight,
                    currentPose.getRotation().times(odometryWeight)
                            .plus(visionPose.getRotation().times(visionWeight)));
            m_swerve.setPose(blendedPose);
        } else {
            DriverStation.reportWarning("Vision pose is null.", false);
        }

        // Send telemetry to the dashboard
        SmartDashboard.putNumber("AprilTag Distance", distance);
        SmartDashboard.putNumber("Angle Error (deg)", Units.radiansToDegrees(angleError));
        SmartDashboard.putNumber("Linear Speed", linearSpeed);
        SmartDashboard.putNumber("Angular Speed", angularSpeed);
        SmartDashboard.putNumber("Tag ID", closestTag.fiducialID);
        SmartDashboard.putString("Status", "Following tag " + fiducialID);
        SmartDashboard.putNumber("Tag Height", tagHeight);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(0, 0), 0, false, true);
        LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.limelightName);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void configureLimelight() {
        LimelightHelpers.setCropWindow(Constants.VisionConstants.limelightName, 0.0, 1.0, 0.0, 1.0);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limelightName,
                0.0, 0.0, Constants.VisionConstants.cameraHeightMeters,
                0.0, Constants.VisionConstants.cameraPitchRadians, 0.0);
    }

    private LimelightTarget_Fiducial selectClosestTag(LimelightTarget_Fiducial[] tags) {
        LimelightTarget_Fiducial closest = tags[0];
        double minTx = Math.abs(closest.tx_pixels);
        double minArea = closest.ta;
        for (LimelightTarget_Fiducial tag : tags) {
            double tx = Math.abs(tag.tx_pixels);
            if (tx < minTx) {
                closest = tag;
                minTx = tx;
                minArea = tag.ta;
            } else if (tx == minTx && tag.ta > minArea) {
                closest = tag;
                minArea = tag.ta;
            }
        }
        return closest;
    }

    private double calculateDistance(double ty, double tagHeight) {
        double angle = Units.degreesToRadians(ty);
        double heightDiff = tagHeight - Constants.VisionConstants.cameraHeightMeters;
        return heightDiff / Math.tan(angle + Constants.VisionConstants.cameraPitchRadians);
    }

    private Pose2d getAprilTagPose(int tagID) {
        if (tagID >= 1 && tagID <= Constants.FieldConstants.aprilTagPoses.length)
            return Constants.FieldConstants.aprilTagPoses[tagID - 1];
        return null;
    }

    private double getAprilTagHeight(int tagID) {
        return Constants.aprilTagHeights.getOrDefault(tagID,
                Constants.VisionConstants.defaultTargetHeightMeters);
    }

    private boolean isStrategicAprilTag(int tagID) {
        switch (tagID) {
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 13:
            case 14:
                return true;
            default:
                return false;
        }
    }

    private boolean willCollideWithFieldElement(ChassisSpeeds chassisSpeeds, Pose2d targetPose) {
        Pose2d currentPose = m_swerve.getPose();
        double robotWidth = Constants.SwerveConstans.kTrackWidth;
        Translation2d currentPos = currentPose.getTranslation();
        Translation2d targetPos = targetPose.getTranslation();
        Translation2d direction = targetPos.minus(currentPos);
        double distanceToTarget = currentPos.getDistance(targetPos);
        double norm = direction.getNorm();
        if (norm > 0)
            direction = direction.div(norm);
        else
            return false;
        Translation2d predictedPos = currentPos.plus(direction.times(distanceToTarget));

        double coralRadius = Constants.FieldConstants.kCoralStationRadius;
        double reefRadius = Constants.FieldConstants.kReefRadius;

        Translation2d[] coralPositions = new Translation2d[] {
                new Translation2d(3.6520, 24.144),
                new Translation2d(3.6520, 7.519),
                new Translation2d(5.3049, 12.997),
                new Translation2d(5.4687, 15.830),
                new Translation2d(5.3049, 18.663),
                new Translation2d(4.9777, 18.663),
                new Translation2d(4.8139, 15.830),
                new Translation2d(4.9777, 12.997),
                new Translation2d(3.32568, 24.144),
                new Translation2d(3.25568, 7.519)
        };
        Translation2d reefPos = new Translation2d(4.5, 15.830);

        for (Translation2d pos : coralPositions) {
            if (predictedPos.getDistance(pos) < (coralRadius + robotWidth / 2.0)) {
                return true;
            }
        }
        if (predictedPos.getDistance(reefPos) < (reefRadius + robotWidth / 2.0)) {
            return true;
        }
        double buffer = 0.5;
        if (predictedPos.getX() < buffer ||
                predictedPos.getX() > Constants.FieldConstants.kFieldLength - buffer ||
                predictedPos.getY() < buffer ||
                predictedPos.getY() > Constants.FieldConstants.kFieldWidth - buffer) {
            return true;
        }
        return false;
    }
}