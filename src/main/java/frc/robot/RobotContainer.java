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

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftX.value;
    private final int strafeAxis = XboxController.Axis.kLeftY.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();

    /* Path Planner */
    public SendableChooser<String> autoChooser;
    public SendableChooser<PathPlannerAuto> autoChooserGame;
    public SendableChooser<PathPlannerAuto> autoChooserTesting;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        autoSelector();
    }

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

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    private void autoSelector() {
        autoChooser = new SendableChooser<String>();
        autoChooserGame = new SendableChooser<PathPlannerAuto>();
        autoChooserTesting = new SendableChooser<PathPlannerAuto>();

        autoChooserTesting.setDefaultOption("Checking OffSet (Be readdy! HellHole)", new PathPlannerAuto("OffSet"));
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
}
