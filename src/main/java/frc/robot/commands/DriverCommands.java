package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class DriverCommands extends Command {

    Swerve m_Swerve = new Swerve();

    public DriverCommands(Swerve Swerve) {
        this.m_Swerve = Swerve;
        addRequirements(m_Swerve);
    }    

    public void newHeading() {
        m_Swerve.setHeading(m_Swerve.getHeading());
    }
}