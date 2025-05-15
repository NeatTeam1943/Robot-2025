package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveConstans;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algea;
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
    private BooleanEvent m_magnetSwitchPressed;
    // private NoLimitSwitchElevatorMoveToLevelXCommand m_elevatorDefaultCommand;

    /**
     * Configure default commands for subsystems
     */

    private void configureDefaultCommands() {
        m_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_Swerve,
                        () -> m_DriveController.getRawAxis(strafeAxis),
                        () -> m_DriveController.getRawAxis(translationAxis),
                        () -> -m_DriveController.getRawAxis(rotationAxis),
                        () -> m_DriveController.leftBumper().getAsBoolean()));

        m_Elevator.setDefaultCommand(new MoveEleveatorTestMagnetHeight(m_Elevator,
                m_MechController));
        // m_Elevator.setDefaultCommand(m_elevatorDefaultCommand);
        m_AlgeaRotatorAxis.setDefaultCommand(new AlgeaRotatorAxisCommand(m_AlgeaRotatorAxis, m_MechController));
        m_Climber.setDefaultCommand(new ClimberMoveCommandTest(m_Climber, m_DriveController));
        m_Coral.setDefaultCommand(new AutoCoralIntakeCommand(m_Coral, m_LedController, m_DriveController,
                m_MechController));
        SmartDashboard.putString("AutoSpeed", SwerveConstans.kMaxAutoVelocity + "");
    }

    /* Path Planner */
    public SendableChooser<String> m_AutoChooser;
    public SendableChooser<PathPlannerAuto> m_AutoChooserGame;
    public SendableChooser<PathPlannerAuto> m_AutoChooserTesting;

    // Chosses an auto for Path Planner
    private void autoSelector() {
        m_AutoChooser = new SendableChooser<String>();
        m_AutoChooserGame = new SendableChooser<PathPlannerAuto>();
        m_AutoChooserTesting = new SendableChooser<PathPlannerAuto>();

        m_AutoChooserTesting.setDefaultOption("None", new PathPlannerAuto("None"));

        /* Test Autos */

        m_AutoChooserTesting.addOption("Forward Middle", new PathPlannerAuto("ForwardMiddle"));
        m_AutoChooserTesting.addOption("Forward Bottom", new PathPlannerAuto("BottomForword"));
        m_AutoChooserTesting.addOption("Rotation", new PathPlannerAuto("Rot"));
        m_AutoChooserTesting.addOption("Rotation and then forword", new PathPlannerAuto("RotThenForword"));
        m_AutoChooserTesting.addOption("Forword with Rotation", new PathPlannerAuto("ForwordWithRot"));
        m_AutoChooserTesting.addOption("Move in a circle", new PathPlannerAuto("Circle"));
        m_AutoChooserTesting.addOption("Doing an S", new PathPlannerAuto("S"));
        m_AutoChooserTesting.addOption("Test Auto", new PathPlannerAuto("TestAuto"));
        m_AutoChooserTesting.addOption("To the Riff with S", new PathPlannerAuto("Check"));
        m_AutoChooserTesting.addOption("Checking OffSet (Be readdy! HellHole)", new PathPlannerAuto("OffSet"));

        /* Game Autos */

        // Upper Autos
        m_AutoChooserGame.setDefaultOption("RunAwayUp", new PathPlannerAuto("RunAwayUp"));
        m_AutoChooserGame.addOption("MaxL1Up", new PathPlannerAuto("MaxL1Up"));
        m_AutoChooserGame.addOption("OneCoralUp", new PathPlannerAuto("OneCoralUp"));
        m_AutoChooserGame.addOption("BestCoralUp", new PathPlannerAuto("BestCoralUp"));

        // Down Autos
        m_AutoChooserGame.addOption("RunAwayDown", new PathPlannerAuto("RunAwayDown"));
        m_AutoChooserGame.addOption("MaxL1Down", new PathPlannerAuto("MaxL1Down"));
        m_AutoChooserGame.addOption("OneCoralDown", new PathPlannerAuto("OneCoralDown"));
        m_AutoChooserGame.addOption("BestCoralDown", new PathPlannerAuto("BestCoralDown"));

        // Middel To Up
        m_AutoChooserGame.addOption("RunAwayFromMiddelToUp", new PathPlannerAuto("RunAwayFromMiddelToUp"));
        m_AutoChooserGame.addOption("MaxL1UpFromMiddel", new PathPlannerAuto("MaxL1UpFromMiddel"));
        m_AutoChooserGame.addOption("OneCoralFromMiddelToUp", new PathPlannerAuto("OneCoralFromMiddelToUp"));
        m_AutoChooserGame.addOption("BestCoralUpFromMiddel", new PathPlannerAuto("BestCoralUpFromMiddel"));

        // Middel To Down
        m_AutoChooserGame.addOption("RunAwayFromMiddelToDown", new PathPlannerAuto("RunAwayFromMiddelToDown"));
        m_AutoChooserGame.addOption("MaxL1DownFromMiddel", new PathPlannerAuto("MaxL1DownFromMiddel"));
        m_AutoChooserGame.addOption("OneCoralFromMiddelToDown", new PathPlannerAuto("OneCoralFromMiddelToDown"));
        m_AutoChooserGame.addOption("BestCoralDownFromMiddel", new PathPlannerAuto("BestCoralDownFromMiddel"));

        m_AutoChooser.setDefaultOption("Auto Chooser Game", "autoChooserGame");
        m_AutoChooser.setDefaultOption("Auto Chooser Testing", "autoChooserTesting");

        /* Once Coral Middel */

        m_AutoChooserGame.addOption("FW&L3", new PathPlannerAuto("FW&L3"));

        SmartDashboard.putData("Auto Chooser", m_AutoChooser);

    }

    // Returns the PP Auto for the auto time in the game
    public Command getAutonomousCommand() {
        switch (m_AutoChooser.getSelected()) {
            case "autoChooserTesting":
                return m_AutoChooserTesting.getSelected();

            default:
            case "autoChooserGame":
                return m_AutoChooserGame.getSelected();
        }
    }

    /* Driver Buttons */
    private final CommandXboxController m_DriveController;
    private final CommandXboxController m_MechController;

    /* Subsystems */
    public final Swerve m_Swerve;
    private Coral m_Coral;
    private Elevator m_Elevator;
    private Climber m_Climber;
    private Algea m_AlgeaRotatorAxis;
    public LedController m_LedController;

    /* Swerve */
    private final int rotationAxis;
    private final int strafeAxis;
    private final int translationAxis;

    /* Tag Command */

    private TagCommand m_TagCommand;

    public String getThroBore() {
        return m_Elevator.encoderValue() + "";
    }

    public void resetElevator() {
        m_Elevator.resetEncoderValue();
    }

    public RobotContainer() {
        /* Camera */
        CameraServer.startAutomaticCapture("camera", 0);
        CameraServer.startAutomaticCapture("camera 1", 1);

        m_DriveController = new CommandXboxController(0);
        m_MechController = new CommandXboxController(1);
        /* Drive Controls */
        translationAxis = XboxController.Axis.kLeftX.value;
        strafeAxis = XboxController.Axis.kLeftY.value;
        rotationAxis = XboxController.Axis.kRightX.value;

        /* Subsystems */
        m_Swerve = new Swerve();
        m_Coral = new Coral();
        m_Elevator = new Elevator();
        m_LedController = new LedController();
        m_AlgeaRotatorAxis = new Algea();
        m_Climber = new Climber();
        // m_Algea = new Algea();

        m_TagCommand = new TagCommand(m_Swerve);
        NamedCommands.registerCommand("CoralCommand", new CoralCommand(m_Coral, m_LedController));
        NamedCommands.registerCommand("Reset Elevator",
                new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 0, m_LedController));
        NamedCommands.registerCommand("Elevator L2",
                new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 2, m_LedController));
        NamedCommands.registerCommand("Elevator L3",
                new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 3, m_LedController));
        NamedCommands.registerCommand("Elevator L4", new ElevatorFullExtend(m_Elevator));

        m_magnetSwitchPressed = new BooleanEvent(Robot.getEventLoop(),
                m_Elevator::ElevatorBottomMagnetSwitchState);

        m_magnetSwitchPressed.ifHigh(() -> m_Elevator.resetEncoderValue());

        configureDefaultCommands();
        configureButtonBindings();
        autoSelector();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        m_DriveController.a().whileTrue(new InstantCommand(() -> m_Swerve.zeroHeading()));
        m_DriveController.b().whileTrue(new ClimberCommand(m_Climber, true));
        m_DriveController.rightBumper().onTrue(new RunCommand(
                () -> m_LedController.setLedColor(BlinkinPattern.Gold), m_LedController));
        m_DriveController.rightBumper()
                .onFalse(new RunCommand(() -> m_LedController.setToDefaultColor(), m_LedController));

        /* Mech Buttons */
        // Coral
        m_MechController.y().whileTrue(new CoralCommand(m_Coral, m_LedController)); // Coral Out
        m_MechController.back().onTrue(new ReInsert(m_Coral, m_LedController));
        m_MechController.x().onTrue(new CoralCommand(m_Coral, m_LedController) // Coral Out
                .andThen(new ResetElevatorCommand(m_Elevator, m_LedController))); // And then Elevator Down

        // Elevator
        m_MechController.povDown()
                .onTrue(new ResetElevatorCommand(m_Elevator, m_LedController));
        m_MechController.povLeft()
                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 1, m_LedController));
        m_MechController.povRight()
                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 2,
                        m_LedController));
        m_MechController.povUp()
                .onTrue(new NoLimitSwitchElevatorMoveToLevelXCommand(m_Elevator, 3, m_LedController));
        m_MechController.a().onTrue(new InstantCommand(() -> m_Elevator.resetEncoderValue(), m_Elevator));
        m_DriveController.leftBumper().whileTrue(new TagCommand(m_Swerve));
        m_DriveController.y().whileTrue(new FollowTagCommand(m_Swerve));

    }

    public void setToNeatTeamLED() {
        m_LedController.ColorSetter(BlinkinPattern.DontDoThisNeatTeam);
    }

    public Swerve getSwerve() {
        return m_Swerve;
    }

    public Elevator getElevator() {
        return m_Elevator;
    }

    public Algea getALgea() {
        return m_AlgeaRotatorAxis;
    }

    public Coral getCoral() {
        return m_Coral;
    }

    public void resetGyro() {
        m_Swerve.gyro.reset();
    }

    public Rotation2d getGyroYaw() {
        return m_Swerve.getGyroYaw();
    }
}
