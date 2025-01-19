package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.exampleAuto;
=======
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
>>>>>>> Stashed changes
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
<<<<<<< Updated upstream
    private final Joystick driver = new Joystick(0);
    private final XboxController controller = new XboxController(0);
=======
    public CommandXboxController m_DriveController = new CommandXboxController(Constants.OperatorConstants.kDriveControllerPort);
    public CommandXboxController m_MechController;
    private final Joystick driver = new Joystick(0);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
>>>>>>> Stashed changes

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
<<<<<<< Updated upstream
    private final Swerve s_Swerve = new Swerve();

=======
    public  final Swerve s_Swerve = new Swerve();
	@SuppressWarnings("unused")
    private DriverCommands driverCommands = new DriverCommands(s_Swerve);   
	private CoralIntake m_Coralintake;
    private CoralOutTake m_CoralOutTake;
    private Algea m_Algea;
    private Elevator m_Elevator;
>>>>>>> Stashed changes
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    /**
     * Configure default commands for subsystems
     */
    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
<<<<<<< Updated upstream
=======
        m_DriveController.a().whileTrue(new RunCommand(() -> s_Swerve.zeroHeading(), s_Swerve));
        m_MechController = new CommandXboxController(Constants.OperatorConstants.kMechanisemControllerPort);
        m_Algea = new Algea();
        m_Coralintake = new CoralIntake();
        m_CoralOutTake = new CoralOutTake();
        m_Elevator = new Elevator();
            
        // Configure the button bindings
        configureButtonBindings();
>>>>>>> Stashed changes
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
<<<<<<< Updated upstream
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
=======

        m_DriveController.a().whileTrue(new RunCommand(() -> s_Swerve.zeroHeading(), s_Swerve));
        m_MechController.y().whileTrue(new CoralTransportCommand(m_Coralintake));
        m_MechController.y().whileTrue(new CoralTransportOutTake(m_CoralOutTake));
        m_MechController.x().whileTrue(new CoralOutTakeCommand(m_CoralOutTake));
        m_MechController.a().whileTrue(new AlgeaInCommand(m_Algea));
        m_MechController.b().whileTrue(new AlgeaOutCommand(m_Algea));
        m_MechController.start().onTrue(new ElevatorResetCommand(m_Elevator));
        
        if(m_MechController.rightBumper().getAsBoolean()){
            if(m_MechController.povDown().getAsBoolean()){
                m_MechController.rightBumper().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator , 1));
            }
            else if(m_MechController.povLeft().getAsBoolean()){
                m_MechController.rightBumper().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator , 2));
            }
            else if(m_MechController.povUp().getAsBoolean()){
                m_MechController.rightBumper().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator , 3));

            }
            else if(m_MechController.povRight().getAsBoolean()){
                m_MechController.rightBumper().onTrue(new ElevatorMoveXlevelsCommand(m_Elevator , 4));
            }
            else{
                System.out.println("You are trying to confirm the selected level to the elvator but it seems like there is no selected level");
            }
        }


            
        
>>>>>>> Stashed changes
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new exampleAuto(s_Swerve);
    }
}
