package frc.lib.neat;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class DigitalSensor implements BooleanSupplier {

  private final DigitalInput input;
  private final Trigger trigger;

  public DigitalSensor(final int channel) {
    input = new DigitalInput(channel);
    trigger = new Trigger(input::get);
  }

  public final DigitalInput input() {
    return input;
  }

  public final Trigger trigger() {
    return trigger;
  }

  @Override
  public boolean getAsBoolean() {
    return input.get();
  }
}
