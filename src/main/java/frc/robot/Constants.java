package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import java.util.HashMap;
import java.util.Map;

public final class Constants {

    public static final double kStickDeadband = 0.1;

    public static final class OperatorConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kMechanisemControllerPort = 1;

    }

    public static final class Swerve {

        public static final int kPigeonID = 1;

        public static final COTSTalonFXSwerveConstants kChosenModule = // TODO: This must be tuned to specific
                                                                       // robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double kTrackWidth = Units.inchesToMeters(27); // TODO: This must be tuned to
        // specific
        // robot
        public static final double kWheelBase = Units.inchesToMeters(22); // TODO: This must be tuned to
        // specific
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

        /* Module Gear Ratios */
        public static final double kDriveGearRatio = kChosenModule.driveGearRatio;
        public static final double kAngleGearRatio = 18.75 / 1;

        /* Motor Inverts */
        public static final InvertedValue kAngleMotorInvert = kChosenModule.angleMotorInvert;
        public static final InvertedValue kDriveMotorInvert = kChosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue kCancoderInvert = kChosenModule.cancoderInvert;

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
        public static final double kDriveKP = 0.005; // TODO: This must be tuned to specific robot (was 0.12)
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double kDriveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double kDriveKV = 1.9151148816283379;
        public static final double kDriveKA = 0.27;

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public static final double kMaxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public static final double kMaxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelCircumference = kChosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = kChosenModule.driveGearRatio;
        public static final double angleGearRatio = 18.75 / 1;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = kChosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = kChosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = kChosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = kChosenModule.angleKP;
        public static final double angleKI = kChosenModule.angleKI;
        public static final double angleKD = kChosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;
        // Angle Motor Stator Current Limiting
        public static final boolean angleEnableStatorCurrentLimit = true; // Enable or disable stator current limiting
        public static final double angleStatorCurrentLimit = 40.0; // Stator current limit in Amperes
        public static final double angleStatorCurrentThreshold = 45.0; // Threshold current in Amperes to trigger
                                                                       // limiting
        public static final double angleStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is applied

        // Drive Motor Stator Current Limiting
        public static final boolean driveEnableStatorCurrentLimit = true; // Enable or disable stator current limiting
        public static final double driveStatorCurrentLimit = 50.0; // Stator current limit in Amperes
        public static final double driveStatorCurrentThreshold = 55.0; // Threshold current in Amperes to trigger
                                                                       // limiting
        public static final double driveStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is applied

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double maxSpeed = Double.parseDouble(SmartDashboard.getString("DB/String 0", "4.5")); // TODO:
                                                                                                            // This must
                                                                                                            // be tuned
                                                                                                            // to
                                                                                                            // specific
                                                                                                            // robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // A1
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        // B2
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-64.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        // B1
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(96);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        // A2
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(175.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static int getKpigeonid() {
            return kPigeonID;
        }

        public static COTSTalonFXSwerveConstants getkChosenModule() {
            return kChosenModule;
        }

        public static double getKtrackwidth() {
            return kTrackWidth;
        }

        public static double getKwheelbase() {
            return kWheelBase;
        }

        public static double getKwheelcircumference() {
            return kWheelCircumference;
        }

        public static SwerveDriveKinematics getKswervekinematics() {
            return kSwerveKinematics;
        }

        public static double getKdrivegearratio() {
            return kDriveGearRatio;
        }

        public static double getKanglegearratio() {
            return kAngleGearRatio;
        }

        public static InvertedValue getKanglemotorinvert() {
            return kAngleMotorInvert;
        }

        public static InvertedValue getKdrivemotorinvert() {
            return kDriveMotorInvert;
        }

        public static SensorDirectionValue getKcancoderinvert() {
            return kCancoderInvert;
        }

        public static int getKanglecurrentlimit() {
            return kAngleCurrentLimit;
        }

        public static int getKanglecurrentthreshold() {
            return kAngleCurrentThreshold;
        }

        public static double getKanglecurrentthresholdtime() {
            return kAngleCurrentThresholdTime;
        }

        public static boolean isKangleenablecurrentlimit() {
            return kAngleEnableCurrentLimit;
        }

        public static int getKdrivecurrentlimit() {
            return kDriveCurrentLimit;
        }

        public static int getKdrivecurrentthreshold() {
            return kDriveCurrentThreshold;
        }

        public static double getKdrivecurrentthresholdtime() {
            return kDriveCurrentThresholdTime;
        }

        public static boolean isKdriveenablecurrentlimit() {
            return kDriveEnableCurrentLimit;
        }

        public static double getKopenloopramp() {
            return kOpenLoopRamp;
        }

        public static double getKclosedloopramp() {
            return kClosedLoopRamp;
        }

        public static double getKanglekp() {
            return kAngleKP;
        }

        public static double getKangleki() {
            return kAngleKI;
        }

        public static double getKanglekd() {
            return kAngleKD;
        }

        public static double getKdrivekp() {
            return kDriveKP;
        }

        public static double getKdriveki() {
            return kDriveKI;
        }

        public static double getKdrivekd() {
            return kDriveKD;
        }

        public static double getKdrivekf() {
            return kDriveKF;
        }

        public static double getKdriveks() {
            return kDriveKS;
        }

        public static double getKdrivekv() {
            return kDriveKV;
        }

        public static double getKdriveka() {
            return kDriveKA;
        }

        public static double getKmaxspeed() {
            return kMaxSpeed;
        }

        public static double getKmaxangularvelocity() {
            return kMaxAngularVelocity;
        }

        public static NeutralModeValue getKangleneutralmode() {
            return kAngleNeutralMode;
        }

        public static NeutralModeValue getKdriveneutralmode() {
            return kDriveNeutralMode;
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
        // tuned to specific robot

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

    // TODO : CHANGE PORTS AND VALUES TO ANYTHING BELOW HERE OR ROBOT GO
    // BOOOOOOM!!!!!!
    public static final class CoralConstants {

        public static final int kPhotoSwitchPort = 4;
        public static final int kLeftMotorPort = 50;
        public static final int kRightMotorPort = 51;
        public static final double kCoralOutSpeed = -0.7;
        public static final double kCoralInSpeed = -0.1;
    }

    public static final class AlgeaConstants {

        public static final int kMotorPort = 55;
        public static final int kPhotoSwitchPort = 6;

    }

    public static final class MotorCurrentLimits {

        public static final int kSupplyCurrentLimit = 40;
        public static final boolean kSupplyCurrentLimitEnable = true;
    }

    public static final class ElevatorConstants {

        public static final int kMagnetSwitchPort = 0;
        public static final int kRightMotorPort = 52;
        public static final int kLeftMotorPort = 53;
        public static final int kBottomLimitSwitchPort = 1;
        public static final int kTopLimitSwitchPort = 2;
        public static final double kL1EncoderValue = 0.2;
        public static final double kL2EncoderValue = 0.4;
        public static final double kL3EncoderValue = 0.6;
        public static final double kL4EncoderValue = 0.8;
        public static final double kEncoderValueTreshHold = 0.1;

    }

    public static final class AlgeaRotatorAxisConstants {

        public static final int kTopLimitSwichPort = 5;
        public static final int kBottomLimitSwitchPort = 3;
        public static final int kMotorPort = 54;
        public static final double kEncoderValueForElevatorReset = 5;
        public static final double kEncoderValueForElevatorL1 = 10;
        public static final double kEncoderValueLimit = 50;

    }

    public static final class LedConstants {
        public static final int kBlinkinControllerPort = 0;
        public static final double kDefualtColor = 0.85;
        public static final double kMovingElevatorColor = -0.99;
        public static final double kAtWantedLevelColor = 0.57;
        public static final double kCoralInColor = 0.93;
    }

    public static final class VisionConstants {
        public static final String limelightName = "limelight-nt";
        public static final double cameraHeightMeters = 0.3; // m
        public static final double cameraPitchRadians = Units.degreesToRadians(50.0); // rad
        public static final double defaultTargetHeightMeters = 0.9144; // m fallback
        public static final double focalLengthPixels = 320.0; // pixels
        public static final double maxAprilTagDistance = 3.0; // m
        public static final double minAprilTagDistance = 0.1; // m
        public static final double slowdownDistance = 1.0; // m

        // PID gains (from Swerve constants)
        public static final double driveKP = Swerve.kDriveKP;
        public static final double driveKI = Swerve.kDriveKI;
        public static final double driveKD = Swerve.kDriveKD;
        public static final double angleKP = Swerve.kAngleKP;
        public static final double angleKI = Swerve.kAngleKI;
        public static final double angleKD = Swerve.kAngleKD;

        public static final double maxSpeed = 1.5; // m/s
        public static final double minSpeed = 0.2; // m/s

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
        public static final Map<Integer, Double> aprilTagHeights = new HashMap<>() {
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
    }
}
