package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.LimelightHelpers;

public class Swerve extends SubsystemBase {
        public SwerveDrivePoseEstimator swerveOdometry;
        public SwerveModule[] m_SwerveMods;
        public Pigeon2 gyro;
        private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.kModuleTranslations);

        public Swerve() {
                gyro = new Pigeon2(Constants.Swerve.kPigeonID);
                Pigeon2Configuration config = new Pigeon2Configuration();
                config.MountPose.MountPoseYaw = -90; // Adjust if your Pigeon is mounted at an angle
                gyro.getConfigurator().apply(config);
                gyro.setYaw(0);

                m_SwerveMods = new SwerveModule[] {
                                new SwerveModule(0, Constants.Swerve.Mod0.kConstants),
                                new SwerveModule(1, Constants.Swerve.Mod1.kConstants),
                                new SwerveModule(2, Constants.Swerve.Mod2.kConstants),
                                new SwerveModule(3, Constants.Swerve.Mod3.kConstants)
                };

                swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.kSwerveKinematics, getGyroYaw(),
                                getModulePositions(), new Pose2d());

                AutoBuilder.configure(
                                this::getPose,
                                this::setPose,
                                this::getRobotRelativeSpeeds,
                                this::runPureVelocity,
                                new PPHolonomicDriveController(
                                                new PIDConstants(7, 0.0, 0.0), // Rotation - 0.53 (Not good) TODO: tune
                                                                               // this to the robot
                                                new PIDConstants(0.9, 0.0, 0.0)), // Velocity - 4.55 (Not good) TODO:
                                                                                  // tune this to the robot
                                Constants.Swerve.kPPConfig,
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                this);
        }

        public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
                SwerveModuleState[] swerveModuleStates = Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(
                                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                translation.getX(),
                                                translation.getY(),
                                                rotation,
                                                getHeading())
                                                : new ChassisSpeeds(
                                                                translation.getX(),
                                                                translation.getY(),
                                                                rotation));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeed);

                for (SwerveModule mod : m_SwerveMods) {
                        if (mod == m_SwerveMods[0]) {
                                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
                                double cahnge = 1;// SmartDashboard.getNumber("DB/Slider 0", 0.9);
                                mod.driveVelocity.Velocity /= cahnge;
                        } else {
                                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
                        }
                }
        }

        /* Used by SwerveControllerCommand in Auto */
        public void setModuleStates(SwerveModuleState[] desiredStates) {

                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeed);

                for (SwerveModule mod : m_SwerveMods) {
                        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
                }
        }

        public SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = new SwerveModuleState[4];
                for (SwerveModule mod : m_SwerveMods) {
                        states[mod.moduleNumber] = mod.getState();
                }
                return states;
        }

        public SwerveModulePosition[] getModulePositions() {
                SwerveModulePosition[] positions = new SwerveModulePosition[4];
                for (SwerveModule mod : m_SwerveMods) {
                        positions[mod.moduleNumber] = mod.getPosition();
                }
                return positions;
        }

        public Pose2d getPose() {
                return swerveOdometry.getEstimatedPosition();
        }

        public void setPose(Pose2d pose) {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }

        public Rotation2d getHeading() {
                return getPose().getRotation();
        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
                return kinematics.toChassisSpeeds(getModuleStates());
        }

        public void runPureVelocity(ChassisSpeeds speeds) {
                var moduleStates = kinematics.toSwerveModuleStates(speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.kMaxSpeed);
                setModuleStates(moduleStates);
        }

        public void setHeading(Rotation2d heading) {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                                new Pose2d(getPose().getTranslation(), heading));
        }

        public void zeroHeading() {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                                new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }

        public Rotation2d getGyroYaw() {
                // Convert the Pigeon's yaw angle to a Rotation2d
                return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
        }

        public void resetModulesToAbsolute() {
                for (SwerveModule mod : m_SwerveMods) {
                        mod.resetToAbsolute();
                }
        }

        @Override
        public void periodic() {
                swerveOdometry.update(getGyroYaw(), getModulePositions());

                Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.VisionConstants.limelightName);
                if (visionPose != null) {
                        Pose2d currentPose = getPose();
                        double visionWeight = 0.3;
                        double odometryWeight = 1.0 - visionWeight;
                        Pose2d blendedPose = new Pose2d(
                                        currentPose.getX() * odometryWeight + visionPose.getX() * visionWeight,
                                        currentPose.getY() * odometryWeight + visionPose.getY() * visionWeight,
                                        currentPose.getRotation().times(odometryWeight)
                                                        .plus(visionPose.getRotation().times(visionWeight)));
                        setPose(blendedPose);
                }

                for (SwerveModule mod : m_SwerveMods) {
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
                                        mod.getCANcoder().getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",
                                        mod.getPosition().angle.getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                                        mod.getState().speedMetersPerSecond);
                }
        }
}
