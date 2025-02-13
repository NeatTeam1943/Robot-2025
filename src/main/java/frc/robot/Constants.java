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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
        public static final double stickDeadband = 0.1;

        public static final class Swerve {
                public static final int pigeonID = 1;
                public static final DCMotor driveGearbox = DCMotor.getFalcon500(1);

                public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                              // robot
                                COTSTalonFXSwerveConstants.SDS.MK4i
                                                .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to
                                                                                     // specific robot
                public static final double wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to
                                                                                    // specific robot
                public static final double wheelCircumference = chosenModule.wheelCircumference;

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

                public static final Translation2d[] moduleTranslations = new Translation2d[] {
                                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
                };

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = 18.75 / 1;

                /* Motor Inverts */
                public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
                public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

                // PathPlanner configuration
                public static final double robotMassKg = 40;
                public static final double robotMOI = 1 / 12.0 * robotMassKg * (2 * trackWidth * trackWidth);
                public static final double wheelCOF = 1.2;
                public static final double driveMotorReduction = 5.96;
                public static final int driveMotorCurrentLimit = 50;
                public static final double wheelRadiusMeters = Units.inchesToMeters(0.787402);

                public static final RobotConfig ppConfig = new RobotConfig(
                                robotMassKg,
                                robotMOI,
                                new ModuleConfig(
                                                0.02,
                                                10,
                                                wheelCOF,
                                                driveGearbox.withReduction(driveMotorReduction),
                                                100,
                                                1),
                                moduleTranslations);

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
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;

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
                public static final boolean angleEnableStatorCurrentLimit = true; // Enable or disable stator current
                                                                                  // limiting
                public static final double angleStatorCurrentLimit = 40.0; // Stator current limit in Amperes
                public static final double angleStatorCurrentThreshold = 45.0; // Threshold current in Amperes to
                                                                               // trigger limiting
                public static final double angleStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is
                                                                                  // applied

                // Drive Motor Stator Current Limiting
                public static final boolean driveEnableStatorCurrentLimit = true; // Enable or disable stator current
                                                                                  // limiting
                public static final double driveStatorCurrentLimit = 50.0; // Stator current limit in Amperes
                public static final double driveStatorCurrentThreshold = 55.0; // Threshold current in Amperes to
                                                                               // trigger limiting
                public static final double driveStatorCurrentThresholdTime = 0.1; // Time in seconds before limiting is
                                                                                  // applied

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static double maxSpeed = 4.5; // TODO:
                                                     // This
                                                     // must
                                                     // be
                                                     // tuned
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
}
