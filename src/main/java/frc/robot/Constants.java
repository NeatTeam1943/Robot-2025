package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.Swerve;

public final class Constants {
    public static final double kStickDeadband = 0.1;

    public static final class ControllerPorts {
        public static final int kMechControllerPort = 0;
        public static final int kDriverControllerPort = 1;
    }

    public static final class SwerveConstans {
        public static final int kPigeonID = 1;
        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);

        public static final COTSTalonFXSwerveConstants kChosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double kTrackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                              // robot
        public static final double kWheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double kWheelCircumference = kChosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        public static final Translation2d[] kModuleTranslations = new Translation2d[] {
                new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
                new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
                new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
                new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
        };

        /* Module Gear Ratios */
        public static final double kDriveGearRatio = kChosenModule.driveGearRatio;
        public static final double kAngleGearRatio = 18.75 / 1;

        /* Motor Inverts */
        public static final InvertedValue kAngleMotorInvert = kChosenModule.angleMotorInvert;
        public static final InvertedValue kDriveMotorInvert = kChosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue kCancoderInvert = kChosenModule.cancoderInvert;

        /* PathPlanner configuration */
        public static final double kRobotMassKg = 45;
        public static final double kRobotMOI = 1 / 12.0 * kRobotMassKg * (2 * 0.69 * 0.69);
        public static final double kWheelCOF = 1.2;
        public static final double kDriveMotorReduction = 5.96;
        public static final int kDriveMotorCurrentLimit = 24;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(0.787402);

        public static final double kMaxAutoVelocity = 4.5;

        public static final RobotConfig kPPConfig = new RobotConfig(
                kRobotMassKg,
                kRobotMOI,
                new ModuleConfig(
                        kWheelRadiusMeters,
                        kMaxAutoVelocity,
                        kWheelCOF,
                        kDriveGearbox.withReduction(kDriveMotorReduction),
                        kDriveMotorCurrentLimit,
                        1),
                kModuleTranslations);

        /* Swerve Current Limiting */
        public static final int kAngleCurrentLimit = 25;
        public static final int kAngleCurrentThreshold = 40;
        public static final double kAngleCurrentThresholdTime = 0.1;
        public static final boolean kAngleEnableCurrentLimit = true;

        public static final int kDriveCurrentLimit = 35;
        public static final int kDriveCurrentThreshold = 60;
        public static final double kDriveCurrentThresholdTime = 0.1;
        public static final boolean kDriveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double kAngleKP = kChosenModule.angleKP;
        public static final double kAngleKI = kChosenModule.angleKI;
        public static final double kAngleKD = kChosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double kDriveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double kDriveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double kDriveKV = 1.51;
        public static final double kDriveKA = 0.27;

        // Angle Motor Stator Current Limiting
        public static final boolean kAngleEnableStatorCurrentLimit = true; // Enable or disable stator current limiting
        public static final double kAngleStatorCurrentLimit = 40.0; // Stator current limit in Amperes
        public static final double kAngleStatorCurrentThreshold = 45.0; // Threshold current in Amperes to trigger
                                                                        // limiting
        public static final double kAngleStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is applied

        // Drive Motor Stator Current Limiting
        public static final boolean kDriveEnableStatorCurrentLimit = true; // Enable or disable stator current limiting
        public static final double kDriveStatorCurrentLimit = 24.0; // Stator current limit in Amperes
        public static final double kDriveStatorCurrentThreshold = 55.0; // Threshold current in Amperes to trigger
                                                                        // limiting
        public static final double kDriveStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is applied

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double kMaxSpeed = 4.5;

        /** Radians per Second */
        public static final double kMaxAngularVelocity = 6.283;

        /* Neutral Modes */
        public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // A1
            public static final int kAngleMotorID = 11;
            public static final int kDriveMotorID = 12;
            public static final int kCanCoderID = 13;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(52);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // B2
            public static final int kAngleMotorID = 41;
            public static final int kDriveMotorID = 42;
            public static final int kCanCoderID = 43;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(-64.5);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // B1
            public static final int kAngleMotorID = 21;
            public static final int kDriveMotorID = 22;
            public static final int kCanCoderID = 23;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(96);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // A2
            public static final int kAngleMotorID = 31;
            public static final int kDriveMotorID = 32;
            public static final int kCanCoderID = 33;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(175.5);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class CoralConstants {

        public static final int kPhotoSwitchPort = 9;
        public static final int kIntakePhotoSwitchPort = 8;
        public static final int kMotorPort = 9;
        public static final double kCoralOutSpeed = 0.2;
        public static final double kCoralInSpeed = 0.15;

    }

    public static final class MotorCurrentLimits {

        public static final int kSupplyCurrentLimit = 40;
        public static final boolean kSupplyCurrentLimitEnable = true;
    }

    public static final class ElevatorConstant {

        public static final int kMagnetSwitchPort = 2;
        public static final int kRightMotorPort = 55;
        public static final int kLeftMotorPort = 54;
        public static final double kElevatorMoveSpeed = 0.1;
        public static final double kElevatorMaxMoveSpeed = 0.5;
        public static final double kElevatorDownSpeed = -0.3;
        public static final double kStallSpeed = 0.01;
        public static final int kEncoderPortA = 4; // A - Blue PWM
        public static final int kEncoderPortB = 5; // B - Yellow PWM
        public static final int kBottomLimitSwitchPort = 1;
        public static final int kTopLimitSwitchPort = 5;
        public static final double kClosedEncoderValue = 0;
        public static final double kL1EncoderValue = 31;
        public static final double kL2EncoderValue = 23;
        public static final double kL3EncoderValue = 45;
        public static final double kEncoderValueTreshHold = 2;
        public static final double kTroughBoreRatio = 6 / 2;
    }

    public static final class AlgeaConstans {

        public static final int kMotorPort = 50;
        public static final double kClosedStallSpeed = 0.1;
        public static final double kLowerStallSpeed = 0.1;
        public static final double kUpperstallSpeed = 0.3;
        public static final double kEncoderValueTreshHold = 5;
        public static final double kFullExtendEncoderValue = 0.25;
        public static final double kClosedEncoderValue = 0;
        public static final double kLowerAlgaeEncoderValue = 1000;
        public static final double kUpperALgeaEncoderValue = 2500;
        public static final double kFullyOpenEncoderValue = 0.5;
        public static final double kEncoderValueLimit = 0.55;
        public static final double kAlgeaEncValue = 35;
        public static final double kOpenSpeed = 0.05;

    }

    public static final class LedConstants {
        public static final int kBlinkinControllerPort = 9;
        public static final double kDefualtColor = 0.85;
        public static final double kMovingElevatorColor = -0.99;
        public static final double kAtWantedLevelColor = 0.57;
        public static final double kCoralInColor = 0.93;
    }

    public static final class VisionConstants {


        public static final double kWantedRotation = 0;// D
        public static final double kRoationErrorTolernece = 1;// D
        public static final double kDistanceErrorTolerence = 0.1; // m
        public static final double kWantedDistance = 1; // m 

        public static final String limelightName = "limelight-neat";
        public static final double cameraHeightMeters = 0.38; // m
        public static final double cameraPitchRadians = Units.degreesToRadians(50.0); // rad
        public static final double defaultTargetHeightMeters = .51; // m fallback
        public static final double focalLengthPixels = 320.0; // pixels
        public static final double maxAprilTagDistance = 3.0; // m
        public static final double minAprilTagDistance = 0.1; // m
        public static final double slowdownDistance = 1.0; // m

        // PID gains (from Swerve constants)
        public static final double driveKP = Constants.SwerveConstans.kDriveKP;
        public static final double driveKI = Constants.SwerveConstans.kDriveKI;
        public static final double driveKD = Constants.SwerveConstans.kDriveKD;
        public static final double angleKP = Constants.SwerveConstans.kAngleKP;
        public static final double angleKI = Constants.SwerveConstans.kAngleKI;
        public static final double angleKD = Constants.SwerveConstans.kAngleKD;

        public static final double maxSpeed = 0.3; // m/s
        public static final double minSpeed = 0.0; // m/s
        public static final double MaxRotaionSpeed = 0.3; //D/s
        public static final double MinRotaionSpeed = 0.0;//D/s
    }

    public static final class FieldConstants {
        public static final Pose2d[] aprilTagPoses = new Pose2d[] {
                new Pose2d(657.37 * 0.0254, 25.80 * 0.0254, Rotation2d.fromDegrees(126)),
                new Pose2d(657.37 * 0.0254, 291.20 * 0.0254, Rotation2d.fromDegrees(234)),
                new Pose2d(455.15 * 0.0254, 317.15 * 0.0254, Rotation2d.fromDegrees(270)),
                new Pose2d(365.20 * 0.0254, 241.64 * 0.0254, Rotation2d.fromDegrees(0)),
                new Pose2d(365.20 * 0.0254, 75.39 * 0.0254, Rotation2d.fromDegrees(0)),
                new Pose2d(530.49 * 0.0254, 130.17 * 0.0254, Rotation2d.fromDegrees(300)),
                new Pose2d(546.87 * 0.0254, 158.50 * 0.0254, Rotation2d.fromDegrees(0)),
                new Pose2d(530.49 * 0.0254, 186.83 * 0.0254, Rotation2d.fromDegrees(60)),
                new Pose2d(497.77 * 0.0254, 186.83 * 0.0254, Rotation2d.fromDegrees(120)),
                new Pose2d(481.39 * 0.0254, 158.50 * 0.0254, Rotation2d.fromDegrees(180)),
                new Pose2d(497.77 * 0.0254, 130.17 * 0.0254, Rotation2d.fromDegrees(240)),
                new Pose2d(33.51 * 0.0254, 25.80 * 0.0254, Rotation2d.fromDegrees(54)),
                new Pose2d(33.51 * 0.0254, 291.20 * 0.0254, Rotation2d.fromDegrees(306)),
                new Pose2d(325.68 * 0.0254, 241.64 * 0.0254, Rotation2d.fromDegrees(180)),
                new Pose2d(325.68 * 0.0254, 75.39 * 0.0254, Rotation2d.fromDegrees(180)),
                new Pose2d(235.73 * 0.0254, -0.15 * 0.0254, Rotation2d.fromDegrees(90)),
                new Pose2d(160.39 * 0.0254, 130.17 * 0.0254, Rotation2d.fromDegrees(240)),
                new Pose2d(144.00 * 0.0254, 158.50 * 0.0254, Rotation2d.fromDegrees(180)),
                new Pose2d(160.39 * 0.0254, 186.83 * 0.0254, Rotation2d.fromDegrees(120)),
                new Pose2d(193.10 * 0.0254, 186.83 * 0.0254, Rotation2d.fromDegrees(60)),
                new Pose2d(209.49 * 0.0254, 158.50 * 0.0254, Rotation2d.fromDegrees(0)),
                new Pose2d(193.10 * 0.0254, 130.17 * 0.0254, Rotation2d.fromDegrees(300))
        };

        // Field dimensions (in m)
        public static final double kFieldWidth = Units.inchesToMeters(324.0);
        public static final double kFieldLength = Units.inchesToMeters(648.0);
        public static final double kCoralStationRadius = Units.inchesToMeters(48.0);
        public static final double kReefRadius = Units.inchesToMeters(36.0);
        public static final double kBargeWidth = Units.inchesToMeters(90.0);
    }

    // AprilTag heights (in m) from FE‑2025 drawings using the Z coordinate.
    public static final HashMap<Integer, Double> aprilTagHeights = new HashMap<>() {
        {
            put(1, 58.50 * 0.0254);
            put(2, 58.50 * 0.0254);
            put(3, 51.25 * 0.0254);
            put(4, 73.54 * 0.0254);
            put(5, 73.54 * 0.0254);
            put(6, 12.13 * 0.0254);
            put(7, 12.13 * 0.0254);
            put(8, 12.13 * 0.0254);
            put(9, 12.13 * 0.0254);
            put(10, 12.13 * 0.0254);
            put(11, 12.13 * 0.0254);
            put(12, 58.50 * 0.0254);
            put(13, 58.50 * 0.0254);
            put(14, 73.54 * 0.0254);
            put(15, 73.54 * 0.0254);
            put(16, 51.25 * 0.0254);
            put(17, 12.13 * 0.0254);
            put(18, 12.13 * 0.0254);
            put(19, 12.13 * 0.0254);
            put(20, 12.13 * 0.0254);
            put(21, 12.13 * 0.0254);
            put(22, 12.13 * 0.0254);
        }
    };

    public static final class ClimberConstants {
        public static final int kMotorPort = 8;
        public static final int kLimitSwitchPort = 0;
        public static final double kClimberUpSpeed = 0.1;
        public static final double kClimberDownSpeed = -0.3;
        public static final double kClimberStallSpeed = -0.05; // TODO: this needs to be tune via pid at a later date
    }

}
