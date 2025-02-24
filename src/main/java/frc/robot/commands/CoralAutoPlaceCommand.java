// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LedController;

public class CoralAutoPlaceCommand extends Command {
    private final Coral coral;
    private final LedController ledController;
    private final Timer timer;
    private double timeout;
    private double speed;
    private int targetLevel;

    public CoralAutoPlaceCommand(Coral coral, LedController ledController, int targetLevel) {
        this.coral = coral;
        this.ledController = ledController;
        addRequirements(coral, ledController);
        timer = new Timer();
        this.targetLevel = targetLevel;

        switch (targetLevel) {
            case 1:
                timeout = 2.0;
                break;
            case 2:
                timeout = 2.5;
                break;
            case 3:
                timeout = 3.0;
                break;
            case 4:
                timeout = 3.5;
                break;
            default:
                timeout = 2.5;
                break;
        }
        speed = Constants.CoralConstants.kCoralOutSpeed;
    }

    @Override
    public void initialize() {
        coral.moveCoral(0);
        timer.reset();
        timer.start();
        ledController.ledColorSetter(LedController.BlinkinPattern.SinelonRainbowPalette);
    }

    @Override
    public void execute() {
        coral.moveCoral(speed);
    }

    @Override
    public boolean isFinished() {
        return (!coral.PhotoSwitchMode() || timer.get() > timeout);
    }

    @Override
    public void end(boolean interrupted) {
        coral.moveCoral(0);
        ledController.DefualtColor();
        timer.stop();
    }
}
