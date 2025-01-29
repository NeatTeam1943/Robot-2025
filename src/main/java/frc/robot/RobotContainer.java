package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // public CommandXboxController m_DriveController = new
    // CommandXboxController(Constants.OperatorConstants.kDriveControllerPort);
    public CommandXboxController m_MechController;
    // private final Joystick driver = new
    // Joystick(Constants.OperatorConstants.kDriveControllerPort);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver,
    // XboxController.Button.kY.value);

    /* Subsystems */
    // public final Swerve s_Swerve = new Swerve();
    // @SuppressWarnings("unused")
    private CoralIntake m_Coralintake;
    private CoralOutTake m_CoralOutTake;
    private Algea m_Algea;
    private Elevator m_Elevator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    /**
     * Configure default commands for subsystems
     */
    private void configureDefaultCommands() {
        // s_Swerve.setDefaultCommand(
        // new TeleopSwerve(
        // s_Swerve,
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis),
        // () -> robotCentric.getAsBoolean()
        // )
        // );
        m_MechController = new CommandXboxController(Constants.OperatorConstants.kMechanisemControllerPort);
        m_Algea = new Algea();
        m_Coralintake = new CoralIntake();
        m_CoralOutTake = new CoralOutTake();
        m_Elevator = new Elevator();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // m_DriveController.a().whileTrue(new RunCommand(() -> s_Swerve.zeroHeading(),
        // s_Swerve));
        m_MechController.y().whileTrue(
                new CoralTransportCommand(m_Coralintake).alongWith(new CoralTransportOutTake(m_CoralOutTake)));
        m_MechController.x().whileTrue(new CoralOutTakeCommand(m_CoralOutTake));
        // m_MechController.y().whileTrue(Commands.sequence(
        // new CoralTransportCommand(m_Coralintake);
        // new WaitCommand(0.5);
        // new
        // ));
        m_MechController.a().whileTrue(new AlgeaMoveCommand(m_Algea, 1));
        m_MechController.b().whileTrue(new AlgeaMoveCommand(m_Algea, -1));

        // m_MechController.b().whileTrue(new AlgeaMoveCommand(m_Algea, -1));
        m_MechController.start().onTrue(new ElevatorResetCommand(m_Elevator));
        m_MechController.back().onTrue(new ElevatorFullExtend(m_Elevator));
        // m_MechController.rightBumper().onTrue(new ElevatorMove(m_Elevator));
        // m_MechController.rightBumper().onTrue(new ElevatorMove(m_Elevator));
        m_MechController.rightBumper()
                .onTrue(/* TODO : add Xmove here as before starting */ new CoralOutTakeCommand(m_CoralOutTake)
                        .andThen(new ElevatorResetCommand(m_Elevator)));
        // if(m_MechController.rightBumper().getAsBoolean()){
        m_MechController.povDown().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator, 1));
        m_MechController.povLeft().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator, 2));
        m_MechController.povUp().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator, 3));
        m_MechController.povRight().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator, 4));
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
