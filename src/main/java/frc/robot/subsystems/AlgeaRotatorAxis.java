// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

/** Add your docs here. */
public class AlgeaRotatorAxis extends SubsystemBase {

  private SparkMax m_Motor;

  public AlgeaRotatorAxis() {
    m_Motor = new SparkMax(Constants.AlgeaRotatorAxisConstants.kMotorPort, MotorType.kBrushless);
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
    limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;
  }

  public double EncoderValueGetter() {
    return m_Motor.getAlternateEncoder().getPosition() / 4;
  }

  public void AlgeaRotatorAxisMove(double speed) {
    m_Motor.set(speed);
  }
}
