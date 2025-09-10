package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

        private void configureDefaultCommands() {
                m_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_Swerve,
                                                () -> m_DriveController.getRawAxis(strafeAxis),
                                                () -> m_DriveController.getRawAxis(translationAxis),
                                                () -> -m_DriveController.getRawAxis(rotationAxis),
                                                () -> m_DriveController.leftBumper().getAsBoolean()));
        }

        public Command getAutonomousCommand() {
                // Return the automated SysId sequence for autonomous mode
                return new PathPlannerAuto("1mFW");
        }

        /* Driver Buttons */
        private final CommandXboxController m_DriveController;

        /* Subsystems */
        public final Swerve m_Swerve;

        /* Swerve */
        private final int rotationAxis;
        private final int strafeAxis;
        private final int translationAxis;

        public RobotContainer() {
                /* Camera */
                CameraServer.startAutomaticCapture("camera", 0);
                CameraServer.startAutomaticCapture("camera 1", 1);

                m_DriveController = new CommandXboxController(0);

                /* Drive Controls */
                translationAxis = XboxController.Axis.kLeftX.value;
                strafeAxis = XboxController.Axis.kLeftY.value;
                rotationAxis = XboxController.Axis.kRightX.value;

                /* Subsystems */
                m_Swerve = new Swerve();

                configureDefaultCommands();
                configureButtonBindings();
        }

        private void configureButtonBindings() {
                /* Driver Buttons */
                m_DriveController.a().whileTrue(new InstantCommand(() -> m_Swerve.zeroHeading()));

                m_DriveController.y().and(m_DriveController.povDown())
                                .whileTrue(m_Swerve.sysIdDynamic(Direction.kReverse,
                                                m_Swerve.getAngleSysIdRoutine()));
                m_DriveController.y().and(m_DriveController.povUp())
                                .whileTrue(m_Swerve.sysIdDynamic(Direction.kForward,
                                                m_Swerve.getAngleSysIdRoutine()));
                m_DriveController.x().and(m_DriveController.povDown())
                                .whileTrue(m_Swerve.sysIdQuasistatic(Direction.kReverse,
                                                m_Swerve.getAngleSysIdRoutine()));
                m_DriveController.x().and(m_DriveController.povUp())
                                .whileTrue(m_Swerve.sysIdQuasistatic(Direction.kForward,
                                                m_Swerve.getAngleSysIdRoutine()));

                m_DriveController.a().and(m_DriveController.povDown())
                                .whileTrue(m_Swerve.sysIdDynamic(Direction.kReverse,
                                                m_Swerve.getDriveSysIdRoutine()));
                m_DriveController.a().and(m_DriveController.povUp())
                                .whileTrue(m_Swerve.sysIdDynamic(Direction.kForward,
                                                m_Swerve.getDriveSysIdRoutine()));
                m_DriveController.b().and(m_DriveController.povDown())
                                .whileTrue(m_Swerve.sysIdQuasistatic(Direction.kReverse,
                                                m_Swerve.getDriveSysIdRoutine()));
                m_DriveController.b().and(m_DriveController.povUp())
                                .whileTrue(m_Swerve.sysIdQuasistatic(Direction.kForward,
                                                m_Swerve.getDriveSysIdRoutine()));

        }

        public Swerve getSwerve() {
                return m_Swerve;
        }

        public void resetGyro() {
                m_Swerve.gyro.reset();
        }

        public Rotation2d getGyroYaw() {
                return m_Swerve.getGyroYaw();
        }
}
