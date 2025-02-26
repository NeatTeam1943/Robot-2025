package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgeaRotatorAxis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.Swerve;
import frc.robot.Vision.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        /**
         * Configure default commands for subsystems
         */

        private void configureDefaultCommands() {
                TeleopSwerve teleopSwerveCmd = new TeleopSwerve(
                                m_Swerve,
                                () -> m_DriveController.getRawAxis(strafeAxis),
                                () -> m_DriveController.getRawAxis(translationAxis),
                                () -> -m_DriveController.getRawAxis(rotationAxis),
                                () -> m_DriveController.leftBumper().getAsBoolean(),
                                () -> m_MechController.getRightX(),
                                () -> m_MechController.getRightY());

                m_Swerve.setDefaultCommand(teleopSwerveCmd);

                this.teleopSwerveCommand = teleopSwerveCmd;

                m_Elevator.setDefaultCommand(new MoveEleveatorTestMagnetHieght(m_Elevator, m_MechController));
                m_AlgeaRotatorAxis.setDefaultCommand(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, m_MechController));
                m_Climber.setDefaultCommand(new ClimberMoveCommandTest(m_Climber, m_DriveController));
        }

        /* Path Planner */
        public SendableChooser<String> autoChooser;
        public SendableChooser<PathPlannerAuto> autoChooserGame;
        public SendableChooser<PathPlannerAuto> autoChooserTesting;

        private void autoSelector() {
                // Test Autos
                autoChooser = new SendableChooser<String>();
                autoChooserGame = new SendableChooser<PathPlannerAuto>();
                autoChooserTesting = new SendableChooser<PathPlannerAuto>();

                autoChooserTesting.setDefaultOption("Checking OffSet (Be readdy! HellHole)",
                                new PathPlannerAuto("OffSet"));
                autoChooserTesting.addOption("Move in a circle", new PathPlannerAuto("Circle"));
                autoChooserTesting.addOption("Doing an S", new PathPlannerAuto("S"));
                autoChooserTesting.addOption("To the Riff with S", new PathPlannerAuto("Check"));
                autoChooserGame.addOption("TestAuto", new PathPlannerAuto("TestAuto"));

                // Upper Autos
                autoChooserGame.setDefaultOption("RunAwayUp", new PathPlannerAuto("RunAwayUp"));
                autoChooserGame.addOption("MaxL1Up", new PathPlannerAuto("MaxL1Up"));
                autoChooserGame.addOption("OneCoralUp", new PathPlannerAuto("OneCoralUp"));
                autoChooserGame.addOption("BestCoralUp", new PathPlannerAuto("BestCoralUp"));

                // Down Autos
                autoChooserGame.addOption("RunAwayDown", new PathPlannerAuto("RunAwayDown"));
                autoChooserGame.addOption("MaxL1Down", new PathPlannerAuto("MaxL1Down"));
                autoChooserGame.addOption("OneCoralDown", new PathPlannerAuto("OneCoralDown"));
                autoChooserGame.addOption("BestCoralDown", new PathPlannerAuto("BestCoralDown"));

                // Middel To Up
                autoChooserGame.addOption("RunAwayFromMiddelToUp", new PathPlannerAuto("RunAwayFromMiddelToUp"));
                autoChooserGame.addOption("MaxL1UpFromMiddel", new PathPlannerAuto("MaxL1UpFromMiddel"));
                autoChooserGame.addOption("OneCoralFromMiddelToUp", new PathPlannerAuto("OneCoralFromMiddelToUp"));
                autoChooserGame.addOption("BestCoralUpFromMiddel", new PathPlannerAuto("BestCoralUpFromMiddel"));

                // Middel To Down
                autoChooserGame.addOption("RunAwayFromMiddelToDown", new PathPlannerAuto("RunAwayFromMiddelToDown"));
                autoChooserGame.addOption("MaxL1DownFromMiddel", new PathPlannerAuto("MaxL1DownFromMiddel"));
                autoChooserGame.addOption("OneCoralFromMiddelToDown", new PathPlannerAuto("OneCoralFromMiddelToDown"));
                autoChooserGame.addOption("BestCoralDownFromMiddel", new PathPlannerAuto("BestCoralDownFromMiddel"));

                autoChooser.setDefaultOption("Auto Chooser Game", "autoChooserGame");
                autoChooser.setDefaultOption("Auto Chooser Testing", "autoChooserTesting");

                // AutoTest

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public Command getAutonomousCommand() {
                switch (autoChooser.getSelected()) {
                        case "autoChooserTesting":
                                return autoChooserTesting.getSelected();

                        default:
                        case "autoChooserGame":
                                return autoChooserGame.getSelected();
                }
        }

        /* Driver Buttons */
        private final CommandXboxController m_DriveController;
        private final CommandXboxController m_MechController;

        /**
         * Configure default commands for subsystems
         */
        /* Subsystems */
        private TeleopSwerve teleopSwerveCommand;
        private final Swerve m_Swerve = new Swerve();
        private Coral m_Coral;
        public Elevator m_Elevator;
        private Climber m_Climber;
        @SuppressWarnings("unused")
        private AlgeaRotatorAxis m_AlgeaRotatorAxis;
        private final LedController m_led = new LedController();
        // private Algea m_Algea;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        /* Swerve */
        private final CoralAlignCommand m_autoCoralAlignL1 = new CoralAlignCommand(
                        m_Swerve, m_Coral, m_led, 1, -1);
        private final CoralAlignCommand m_autoCoralAlignL2 = new CoralAlignCommand(
                        m_Swerve, m_Coral, m_led, 2, -1);
        private final CoralAlignCommand m_autoCoralAlignL3 = new CoralAlignCommand(
                        m_Swerve, m_Coral, m_led, 3, -1);
        private final CoralAlignCommand m_autoCoralAlignL4 = new CoralAlignCommand(
                        m_Swerve, m_Coral, m_led, 4, -1);

        private final int rotationAxis;
        private final int strafeAxis;
        private final int translationAxis;

        public boolean getAlgeaSwtich() {
                return m_Coral.PhotoSwitchMode();
        }

        public String getThroBore() {
                return m_Elevator.encoderValue() + "";
        }

        public void resetElevator() {
                m_Elevator.resetEncoderValue();
        }

        // Coral level selection
        public SendableChooser<Integer> coralAutoLevelChooser;

        private void configureCoralAutoChooser() {
                coralAutoLevelChooser = new SendableChooser<>();
                coralAutoLevelChooser.setDefaultOption("Level 1", 1);
                coralAutoLevelChooser.addOption("Level 2", 2);
                coralAutoLevelChooser.addOption("Level 3", 3);
                coralAutoLevelChooser.addOption("Level 4", 4);
                SmartDashboard.putData("Coral Auto Level", coralAutoLevelChooser);
        }

        public RobotContainer() {
                /* Camera */
                CameraServer.startAutomaticCapture("camera", 0);

                m_MechController = new CommandXboxController(Constants.OperatorConstants.kMechanisemControllerPort);
                m_DriveController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
                /* Drive Controls */
                translationAxis = XboxController.Axis.kLeftX.value;
                strafeAxis = XboxController.Axis.kLeftY.value;
                rotationAxis = XboxController.Axis.kRightX.value;

                /* Subsystems */
                m_Coral = new Coral();
                m_Elevator = new Elevator();
                m_AlgeaRotatorAxis = new AlgeaRotatorAxis();
                m_Climber = new Climber();
                // m_Algea = new Algea();
                NamedCommands.registerCommand("CoralCommand", new CoralCommand(m_Coral, m_led));
                NamedCommands.registerCommand("Elevator L1", new ElevatorResetCommand(m_Elevator, m_led));
                NamedCommands.registerCommand("Elevator L2",
                                new ElevatorMoveToLevelXCommand(m_Elevator, 2, m_led));
                NamedCommands.registerCommand("Elevator L3",
                                new ElevatorMoveToLevelXCommand(m_Elevator, 3, m_led));
                NamedCommands.registerCommand("Elevator L4", new ElevatorFullExtend(m_Elevator));

                configureDefaultCommands();
                configureButtonBindings();
                autoSelector();
                configureCoralAutoChooser();
        }

        private void configureButtonBindings() {
                /* Driver Buttons */
                /* Drive Buttons */
                m_DriveController.a().whileTrue(new InstantCommand(() -> m_Swerve.zeroHeading()));
                m_DriveController.b().whileTrue(new ClimberCommand(m_Climber, true));
                // X Mode on ctrlr X button
                m_DriveController.x().onTrue(new SwerveXMode(m_Swerve));

                // Toggle TwinStick Mode on ctrlr Y button
                m_DriveController.y().onTrue(new InstantCommand(() -> m_Swerve.toggleTwinStickMode()));
                m_DriveController.leftStick().onTrue(new InstantCommand(() -> {
                        teleopSwerveCommand.togglePrecisionMode();
                        m_led.ledColorSetter(teleopSwerveCommand.isPrecisionMode()
                                        ? LedController.BlinkinPattern.Blue
                                        : LedController.BlinkinPattern.Red);
                }));
                m_DriveController.rightStick().onTrue(new InstantCommand(() -> {
                        teleopSwerveCommand.setPrecisionMode(true);
                        m_led.ledColorSetter(LedController.BlinkinPattern.Blue);
                }));
                m_DriveController.rightStick().onFalse(new InstantCommand(() -> {
                        teleopSwerveCommand.setPrecisionMode(false);
                        m_led.DefualtColor();
                }));
                m_DriveController.rightBumper().onTrue(new InstantCommand(() -> {
                        if (LimelightHelpers.getTV(VisionConstants.kLimelightName)) {
                                m_Swerve.updateOdometryWithVision();
                                SmartDashboard.putString("Vision Update", "Manual Reset Triggered");
                        }
                }));

                /* Mech Buttons */
                m_MechController.y().whileTrue(new CoralCommand(m_Coral, m_led));
                m_MechController.start().onTrue(new ElevatorResetCommand(m_Elevator, m_led));
                m_MechController.x().onTrue(new CoralCommand(
                                m_Coral, m_led)
                                .andThen(new ElevatorResetCommand(m_Elevator,
                                                m_led)));

                m_MechController.povDown().onTrue(new ElevatorMoveToLevelXCommandV2(m_Elevator, 1, m_led));
                m_MechController.povLeft().onTrue(new ElevatorMoveToLevelXCommandV2(m_Elevator, 2, m_led));
                m_MechController.povRight().onTrue(new ElevatorMoveToLevelXCommandV2(m_Elevator, 3, m_led));
                m_MechController.povUp().onTrue(new ElevatorMoveToLevelXCommandV2(m_Elevator, 4, m_led));

                m_MechController.a().onTrue(new RunCommand(() -> m_Elevator.resetEncoderValue(), m_Elevator));

                m_MechController.leftBumper().and(m_MechController.b()).onTrue(m_autoCoralAlignL1);
                m_MechController.rightBumper().and(m_MechController.b()).onTrue(m_autoCoralAlignL2);
                m_MechController.leftBumper().and(m_MechController.y()).onTrue(m_autoCoralAlignL3);
                m_MechController.rightBumper().and(m_MechController.y()).onTrue(m_autoCoralAlignL4);

                m_MechController.back().onTrue(new InstantCommand(() -> {
                        int level = coralAutoLevelChooser.getSelected();

                        CoralAlignCommand levelCommand = new CoralAlignCommand(
                                        m_Swerve, m_Coral, m_led, level, -1);

                        levelCommand.schedule();
                }));

                m_MechController.b().onTrue(new InstantCommand(() -> {
                        double elevatorPos = m_Elevator.encoderValue();
                        int targetLevel;

                        if (elevatorPos <= Constants.ElevatorConstants.kL1EncoderValue
                                        + Constants.ElevatorConstants.kEncoderValueTreshHold) {
                                targetLevel = 1;
                        } else if (elevatorPos <= Constants.ElevatorConstants.kL2EncoderValue
                                        + Constants.ElevatorConstants.kEncoderValueTreshHold) {
                                targetLevel = 2;
                        } else if (elevatorPos <= Constants.ElevatorConstants.kL3EncoderValue
                                        + Constants.ElevatorConstants.kEncoderValueTreshHold) {
                                targetLevel = 3;
                        } else {
                                targetLevel = 4;
                        }

                        new CoralAlignCommand(m_Swerve, m_Coral, m_led, targetLevel, -1).schedule();
                        SmartDashboard.putNumber("Smart Align Level", targetLevel);
                }));
        }
}
