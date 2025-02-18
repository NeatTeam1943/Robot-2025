package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double kStickDeadband = 0.1;

    public final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kMechanisemControllerPort = 1;
    }

    public static final class Swerve {
        public static final int kPigeonID = 1;
        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);

        public static final COTSTalonFXSwerveConstants kChosenModule = // TODO: This must be tuned to specific
                                                                       // robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double kTrackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to
                                                                              // specific robot
        public static final double kWheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to
                                                                             // specific robot
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

        // PathPlanner configuration
        public static final double kRobotMassKg = 40;
        public static final double kRobotMOI = 1 / 12.0 * kRobotMassKg * (2 * kTrackWidth * kTrackWidth);
        public static final double kWheelCOF = 1.2;
        public static final double kDriveMotorReduction = 5.96;
        public static final int kDriveMotorCurrentLimit = 50;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(0.787402);

        public static final RobotConfig kPPConfig = new RobotConfig(
                kRobotMassKg,
                kRobotMOI,
                new ModuleConfig(
                        0.02,
                        10,
                        kWheelCOF,
                        kDriveGearbox.withReduction(kDriveMotorReduction),
                        100,
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
        public static final boolean kAngleEnableStatorCurrentLimit = true; // Enable or disable stator current
                                                                           // limiting
        public static final double kAngleStatorCurrentLimit = 40.0; // Stator current limit in Amperes
        public static final double kAngleStatorCurrentThreshold = 45.0; // Threshold current in Amperes to
                                                                        // trigger limiting
        public static final double kAngleStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is
                                                                           // applied

        // Drive Motor Stator Current Limiting
        public static final boolean kDriveEnableStatorCurrentLimit = true; // Enable or disable stator current
                                                                           // limiting
        public static final double kDriveStatorCurrentLimit = 50.0; // Stator current limit in Amperes
        public static final double kDriveStatorCurrentThreshold = 55.0; // Threshold current in Amperes to
                                                                        // trigger limiting
        public static final double kDriveStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is
                                                                           // applied

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double kMaxSpeed = 4.5; // TODO:
                                              // This
                                              // must
                                              // be
                                              // tuned
                                              // to
                                              // specific
                                              // robot
        /** Radians per Second */
        public static final double kMaxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // A1
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int kDriveMotorID = 11;
            public static final int kAngleMotorID = 12;
            public static final int kCanCoderID = 13;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(52);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Front Right Module - Module 1 */
        // B2
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int kDriveMotorID = 41;
            public static final int kAngleMotorID = 42;
            public static final int kCanCoderID = 43;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(-64.5);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Back Left Module - Module 2 */
        // B1
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int kDriveMotorID = 21;
            public static final int kAngleMotorID = 22;
            public static final int kCanCoderID = 23;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(96);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
        }

        /* Back Right Module - Module 3 */
        // A2
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int kDriveMotorID = 31;
            public static final int kAngleMotorID = 32;
            public static final int kCanCoderID = 33;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(175.5);
            public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset);
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

        public static final int kPhotoSwitchPort = 9;
        public static final int kMotorPort = 9;
        public static final double kCoralOutSpeed = 0.2;
        public static final double kCoralInSpeed = 0.15;
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
        public static final int kRightMotorPort = 55;
        public static final int kLeftMotorPort = 54;
        public static final double kStallSpeed = 0.04;
        public static final int kBottomLimitSwitchPort = 8;
        public static final int kTopLimitSwitchPort = 7;
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
}
