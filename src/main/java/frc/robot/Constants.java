package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
        public static final double kStickDeadband = 0.1;


        public static final class OperatorConstants{
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
                public static final double kTrackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to
                                                                                     // specific
                                                                                     // robot
                public static final double kWheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to
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
                /** Meters per Second */
                public static final double kMaxSpeed = 4.5; // TODO: This must be tuned to specific robot
                /** Radians per Second */
                public static final double kMaxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

                /* Neutral Modes */
                public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;
                public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
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


        // TODO : CHANGE PORTS TO ANYTHING BELOW HERE OR ROBOT GO BOOOOOOM!!!!!!
        public static final class CoralIntakeConstants{
                public static final int kPhotoSwitchPort = 0;
                public static final int kMotorPort = 0;
        }


        public static final class CoralOutTakeConstants{
                public static final int kPhotoSwitchPort = 0;
                public static final int kMotorPort = 0;
        }

        public static final class AlgeaConstants {
                public static final int kMotorPort = 0;
                public static final int kPhotoSwitchPort = 0;
                
        }

        public static final class ElevatorConstants {
                public static final int kMagnetSwitchPort = 0;
                public static final int kRightMotorPort = 0;
                public static final int kLeftMotorPort = 0;
                public static final int kBottomLimitSwitchPort = 0;
                public static final int kTopLimitSwitchPort = 0; 
                public static final double kL1EncoderValue = 0;
                public static final double kL2EncoderValue = 0;
                public static final double kL3EncoderValue = 0;
                public static final double kL4EncoderValue = 0;
        
                
        }
}
