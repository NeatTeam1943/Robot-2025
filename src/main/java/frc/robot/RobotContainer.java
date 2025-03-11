package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController mechController;
  private final CommandXboxController driveController;

  private final Coral coral;
  private final Elevator elevator;

  public RobotContainer() {
    mechController = new CommandXboxController(Constants.Operator.kMechanismControllerPort);
    driveController = new CommandXboxController(Constants.Operator.kDriveControllerPort);

    coral = new Coral();
    elevator = new Elevator();

    configureDefaults();
    configureBindings();
  }

  private void configureDefaults() {
    elevator.configureDefaults();
  }

  private void configureBindings() {
    coral.configureBindings(mechController.y());
    elevator.configureBindings(
        mechController.povDown(),
        mechController.povLeft(),
        mechController.povRight(),
        mechController.povUp());
  }
}
