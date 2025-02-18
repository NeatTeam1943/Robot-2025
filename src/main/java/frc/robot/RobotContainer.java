package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlgeaMoveCommand;
import frc.robot.commands.AlgeaRotatorAxisCommand;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.ElevatorFullExtend;
import frc.robot.commands.ElevatorMoveToLevelXCommand;
import frc.robot.commands.ElevatorResetCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Algea;
import frc.robot.subsystems.AlgeaRotatorAxis;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.Swerve;

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
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> driver.getRawAxis(strafeAxis),
                                                () -> driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> robotCentric.getAsBoolean()));
        }
        
                /* Path Planner */
                public SendableChooser<String> autoChooser;
                public SendableChooser<PathPlannerAuto> autoChooserGame;
                public SendableChooser<PathPlannerAuto> autoChooserTesting;

        private void autoSelector() {
                autoChooser = new SendableChooser<String>();
                autoChooserGame = new SendableChooser<PathPlannerAuto>();
                autoChooserTesting = new SendableChooser<PathPlannerAuto>();

                autoChooserTesting.setDefaultOption("Checking OffSet (Be readdy! HellHole)",
                                new PathPlannerAuto("OffSet"));
                autoChooserTesting.addOption("Move in a circle", new PathPlannerAuto("Circle"));
                autoChooserTesting.addOption("Doing an S", new PathPlannerAuto("S"));
                autoChooserTesting.addOption("To the Riff with S", new PathPlannerAuto("Check"));

                autoChooserGame.setDefaultOption("RunAwayUp", new PathPlannerAuto("RunAwayUp"));
                autoChooserGame.addOption("BestBottomStart", new PathPlannerAuto("BestBottomStart"));
                autoChooserGame.addOption("WorstBottomStart", new PathPlannerAuto("WorstBottomStart"));
                autoChooserGame.addOption("BottomPassLine", new PathPlannerAuto("BottomPassLine"));
                autoChooserGame.addOption("MaxL1", new PathPlannerAuto("MaxL1"));
                autoChooserGame.addOption("BestCoralUp", new PathPlannerAuto("BestCoralUp"));
                autoChooserGame.addOption("To Riff", new PathPlannerAuto("BestBottom4 (To Riff no S)"));

                autoChooser.setDefaultOption("Auto Chooser Game", "autoChooserGame");
                autoChooser.setDefaultOption("Auto Chooser Testing", "autoChooserTesting");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                CameraServer.startAutomaticCapture("camera", 0);
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
        private final Joystick driver;
        private final JoystickButton zeroGyro;
        private final JoystickButton robotCentric;
        public CommandXboxController m_MechController;


        /**
         * Configure default commands for subsystems
         */
        /* Subsystems */
        private final Swerve s_Swerve;
        private Coral m_Coral;
        private Algea m_Algea;
        private Elevator m_Elevator;
        private AlgeaRotatorAxis m_AlgeaRotatorAxis;
        private LedController m_LedController;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        /* Swerve */
        private final int rotationAxis;
        private final int strafeAxis;
        private final int translationAxis;
        public RobotContainer() {
                m_MechController = new CommandXboxController(Constants.OperatorConstants.kMechanisemControllerPort);
                driver = new Joystick(Constants.OperatorConstants.kDriverControllerPort);
                zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
                robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
                configureDefaultCommands();
                configureButtonBindings();
                /* Drive Controls */
                translationAxis = XboxController.Axis.kLeftX.value;
                strafeAxis = XboxController.Axis.kLeftY.value;
                rotationAxis = XboxController.Axis.kRightX.value;

                /* Subsystems */
                s_Swerve = new Swerve();
                m_Coral = new Coral();
                m_Elevator = new Elevator();
                // m_Algea = new Algea();
                // m_AlgeaRotatorAxis = new AlgeaRotatorAxis();
                m_LedController = new LedController();


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
                zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
                // m_DriveController.a().whileTrue(new RunCommand(() -> s_Swerve.zeroHeading(),
                // s_Swerve));
                m_MechController.y().whileTrue((new CoralCommand(m_Coral,
                                true, m_LedController)));
                m_MechController.x().whileTrue(new CoralCommand(m_Coral,
                                false, m_LedController));

                m_MechController.a().whileTrue(new AlgeaMoveCommand(m_Algea, 1));
                m_MechController.b().whileTrue(new AlgeaMoveCommand(m_Algea, -1));

                // m_MechController.b().whileTrue(new AlgeaMoveCommand(m_Algea, -1));
                m_MechController.start()
                                .onTrue(new ElevatorResetCommand(m_Elevator, m_LedController)
                                                .alongWith(new AlgeaRotatorAxisCommand(
                                                                m_AlgeaRotatorAxis, 1,
                                                                Constants.AlgeaRotatorAxisConstants.kEncoderValueForElevatorReset)));
                // m_MechController.rightBumper().onTrue(new ElevatorMove(m_Elevator));
                // m_MechController.rightBumper().onTrue(new ElevatorMove(m_Elevator));
                m_MechController.back()
                                .onTrue(/* TODO : add Xmove here as before starting */ new CoralCommand(
                                                m_Coral, false, m_LedController)
                                                .andThen(new ElevatorMoveToLevelXCommand(m_Elevator, 1, m_LedController)
                                                                .alongWith(
                                                                                new AlgeaRotatorAxisCommand(
                                                                                                m_AlgeaRotatorAxis, 1,
                                                                                                Constants.AlgeaRotatorAxisConstants.kEncoderValueForElevatorL1))));
                // if(m_MechController.rightBumper().getAsBoolean()){
                m_MechController.povDown().onTrue(new ElevatorMoveToLevelXCommand(m_Elevator, 1, m_LedController)
                                .alongWith(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, 1,
                                                Constants.AlgeaRotatorAxisConstants.kEncoderValueForElevatorL1)));
                m_MechController.povLeft().onTrue(new ElevatorMoveToLevelXCommand(m_Elevator, 2, m_LedController));
                m_MechController.povUp().onTrue(new ElevatorMoveToLevelXCommand(m_Elevator, 3, m_LedController));
                m_MechController.povRight().onTrue(new ElevatorMoveToLevelXCommand(m_Elevator, 4, m_LedController));
                //
                m_MechController.leftBumper().whileTrue(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, 1,
                                Constants.AlgeaRotatorAxisConstants.kEncoderValueLimit));
                m_MechController.rightBumper().whileTrue(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, -1, 0));
                // }
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        // public Command getAutonomousCommand() {
        // return new exampleAuto(s_Swerve);
        // }

}
