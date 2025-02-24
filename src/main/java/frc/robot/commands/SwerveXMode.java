package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveXMode extends Command {
    private final Swerve swerve;

    public SwerveXMode(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setXMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
