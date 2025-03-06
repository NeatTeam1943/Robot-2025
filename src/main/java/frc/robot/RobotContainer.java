package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgeaRotatorAxis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LedController.BlinkinPattern;

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
                m_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_Swerve,
                                                () -> -m_DriveController.getRawAxis(strafeAxis),
                                                () -> -m_DriveController.getRawAxis(translationAxis),
                                                () -> m_DriveController.getRawAxis(rotationAxis),
                                                () -> m_DriveController.leftBumper().getAsBoolean()));

                m_Elevator.setDefaultCommand(new MoveEleveatorTestMagnetHieght(m_Elevator, m_MechController));
                m_AlgeaRotatorAxis.setDefaultCommand(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, m_MechController));
                m_Climber.setDefaultCommand(new ClimberMoveCommandTest(m_Climber, m_DriveController));
        }

        /* Path Planner */
        public SendableChooser<String> autoChooser;
        public SendableChooser<PathPlannerAuto> autoChooserGame;
        public SendableChooser<PathPlannerAuto> autoChooserTesting;

        private void autoSelector() {
                autoChooser = new SendableChooser<String>();
                autoChooserGame = new SendableChooser<PathPlannerAuto>();
                autoChooserTesting = new SendableChooser<PathPlannerAuto>();

                autoChooserTesting.setDefaultOption("None", new PathPlannerAuto("None"));
                
                // Test Autos
                autoChooserTesting.addOption("Forward", new PathPlannerAuto("Forward"));
                autoChooserTesting.addOption("Rotation", new PathPlannerAuto("Rotation"));
                autoChooserTesting.addOption("Rotation and then forword", new PathPlannerAuto("RotThenForword"));
                autoChooserTesting.addOption("Forword with Rotation", new PathPlannerAuto("ForwordWithRot"));
                autoChooserTesting.addOption("Move in a circle", new PathPlannerAuto("Circle"));
                autoChooserTesting.addOption("Doing an S", new PathPlannerAuto("S"));
                autoChooserTesting.addOption("Test Auto", new PathPlannerAuto("TestAuto"));
                autoChooserTesting.addOption("To the Riff with S", new PathPlannerAuto("Check"));
                autoChooserTesting.addOption("Checking OffSet (Be readdy! HellHole)", new PathPlannerAuto("OffSet"));

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
                m_Swerve.zeroGyro();
                switch (autoChooser.getSelected()) {
                        case "autoChooserTesting":
                                m_Swerve.zeroGyro();
                                return autoChooserTesting.getSelected();

                        default:
                        case "autoChooserGame":
                                m_Swerve.zeroGyro();
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
        public final Swerve m_Swerve;
        private Coral m_Coral;
        private Elevator m_Elevator;
        private Climber m_Climber;
        private AlgeaRotatorAxis m_AlgeaRotatorAxis;
        private LedController m_LedController;
        // private Algea m_Algea;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        /* Swerve */
        private final int rotationAxis;
        private final int strafeAxis;
        private final int translationAxis;

        public boolean getAlgeaSwtich() {
                return m_Coral.PhotoSwitchMode();
        }

        public String getThroBore() {
                return m_Elevator.encoderValue() + "";
                // return m_Coral.PhotoSwitchMode() + "";
        }
        // public double getSpeed() {

        // }

        public void resetElevator() {
                m_Elevator.resetEncoderValue();
        }

        public void getHeading() {
                System.out.println(m_Swerve.getPose());
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
                m_Swerve = new Swerve();
                m_Coral = new Coral();
                m_Elevator = new Elevator();
                m_LedController = new LedController();
                m_AlgeaRotatorAxis = new AlgeaRotatorAxis();
                m_Climber = new Climber();
                // m_Algea = new Algea();
                NamedCommands.registerCommand("CoralCommand", new CoralCommand(m_Coral, m_LedController));
                NamedCommands.registerCommand("Elevator L1", new ResetElevatorCommand(m_Elevator, m_LedController));
                NamedCommands.registerCommand("Elevator L2",
                                new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 2, m_LedController));
                NamedCommands.registerCommand("Elevator L3",
                                new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 3, m_LedController));
                NamedCommands.registerCommand("Elevator L4", new ElevatorFullExtend(m_Elevator));

                configureDefaultCommands();
                configureButtonBindings();
                autoSelector();
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link GenericHID} or one of its subclasses
         * ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                /* Drive Buttons */
                m_DriveController.a().whileTrue(new InstantCommand(() -> m_Swerve.zeroHeading()));
                m_DriveController.b().whileTrue(new ClimberCommand(m_Climber, true));

                /* Mech Buttons */
                m_MechController.y().whileTrue(new CoralCommand(m_Coral, m_LedController));
                m_MechController.x().onTrue(new CoralCommand(
                                m_Coral, m_LedController)
                                .andThen(new ResetElevatorCommand(m_Elevator,
                                                m_LedController)));

                m_MechController.povDown()
                                .onTrue(new ResetElevatorCommand(m_Elevator, m_LedController));
                m_MechController.povLeft()
                                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 1, m_LedController));
                m_MechController.povRight()
                                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 2, m_LedController));
                m_MechController.povUp()
                                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 3, m_LedController));
                // m_AlgeaRotatorAxis.AlgeaRotatorAxisMove(1);
                m_MechController.a().onTrue(new InstantCommand(() -> m_Elevator.resetEncoderValue(), m_Elevator));
                m_MechController.back().onTrue(new ReInsert(m_Coral, m_LedController));
        }

        public void NeatTeamLED() {
                m_LedController.ColorSetter(BlinkinPattern.DontDoThisNeatTeam);
        }

        public Swerve GetSwerve(){
                return m_Swerve;
        }

        public Elevator GetEleavotr(){
                return m_Elevator;
        }
}
