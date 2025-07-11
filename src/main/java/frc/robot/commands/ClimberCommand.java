// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
    /** Creates a new CoralOutTakeCommand. */
    private Climber m_Climber;
    private double speed;
    private boolean m_IsUp;

    public ClimberCommand(Climber climber, boolean isUp) {
        m_Climber = climber;
        m_IsUp = isUp;
        speed = m_IsUp ? Constants.ClimberConstants.kClimberUpSpeed : Constants.ClimberConstants.kClimberDownSpeed;
        addRequirements(m_Climber);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Climber.moveClimber(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Climber.moveClimber(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climber.moveClimber(Constants.ClimberConstants.kClimberStallSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
