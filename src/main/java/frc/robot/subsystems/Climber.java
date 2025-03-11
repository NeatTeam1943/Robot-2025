// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkMax m_Motor;

  public Climber() {
    m_Motor = new SparkMax(Constants.ClimberConstants.kMotorPort, MotorType.kBrushless);
  }

  public void moveClimber(double speed) {
    m_Motor.set(speed);
  }
}
