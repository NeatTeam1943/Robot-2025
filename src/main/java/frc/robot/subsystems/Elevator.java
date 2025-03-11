package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.neat.DigitalSensor;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final TalonFX master;
  private final TalonFX follower;

  private final DigitalSensor bottom;

  private final PIDController pid;

  public Elevator() {
    master = new TalonFX(Constants.Elevator.kMasterMotorId);
    follower = new TalonFX(Constants.Elevator.kFollowerMotorId);

    bottom = new DigitalSensor(Constants.Elevator.kBottomMagnetSwitchPort);

    pid = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);

    configureControl();
  }

  private void configureControl() {
    follower.setControl(new Follower(master.getDeviceID(), false));

    pid.setTolerance(Constants.Elevator.kTolerance);
    pid.setIntegratorRange(-Constants.Elevator.kIRange, Constants.Elevator.kIRange);
  }

  public void configureDefaults() {
    setDefaultCommand(reach(Constants.Level.L0));
  }

  public void configureBindings(
      final Trigger l0, final Trigger l1, final Trigger l2, final Trigger l3) {
    l0.onTrue(reach(Constants.Level.L0));
    l1.onTrue(reach(Constants.Level.L1));
    l2.onTrue(reach(Constants.Level.L2));
    l3.onTrue(reach(Constants.Level.L3));

    bottom.trigger().onTrue(runOnce(this::resetMeasurement));
  }

  private double calculate(final double setpoint) {
    final double speed = pid.calculate(measurement(), setpoint);
    return MathUtil.clamp(speed, Constants.Elevator.kMaxDownSpeed, Constants.Elevator.kMaxUpSpeed);
  }

  private double measurement() {
    return (Math.abs(master.getRotorPosition().getValueAsDouble())
        + Math.abs(follower.getRotorPosition().getValueAsDouble()) / 2);
  }

  private void resetMeasurement() {
    master.setPosition(0);
    follower.setPosition(0);
  }

  private Command reach(final Constants.Level level) {
    final double targetMeasurement = Constants.Elevator.kLevelTargetMeasurements.get(level);
    return run(() -> master.set(calculate(targetMeasurement)));
  }
}
