package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.neat.DigitalSensor;
import frc.robot.Constants;

public class Coral extends SubsystemBase {

  private final SparkMax motor;

  private final DigitalSensor intake;
  private final DigitalSensor outtake;

  public Coral() {
    motor = new SparkMax(Constants.Coral.kMotorId, MotorType.kBrushless);

    intake = new DigitalSensor(Constants.Coral.kIntakeSensorPort);
    outtake = new DigitalSensor(Constants.Coral.kOuttakeSensorPort);
  }

  public void configureBindings(final Trigger button) {
    final Trigger outtakeNeg = outtake.trigger().negate();

    // Intake sensor indicates a coral had entered, but the outtake sensor
    // doesn't recognize it -> Move coral forward until recognized.
    intake.trigger().and(outtakeNeg).toggleOnTrue(move().until(outtake));

    // Button is pressed, coral is recognized by outtake sensor
    // -> Move forward until out (or button isn't pressed anymore).
    button.and(outtake).whileTrue(move().until(outtakeNeg));
  }

  private Command move() {
    return new StartEndCommand(() -> motor.set(Constants.Coral.kMotorSpeed), () -> motor.set(0));
  }
}
