// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedController extends SubsystemBase {
  /** Creates a new LedController. */
  Spark m_blinkin;

  public LedController() {
    m_blinkin = new Spark(Constants.LedConstants.kBlinkinControllerPort);
  }

  double m_color = Constants.LedConstants.kDefualtColor;
  public void LedColorSetter(double Color) {
    m_color = Color;
  }

  @Override
  public void periodic() {
    m_blinkin.set(m_color);
    System.out.println(m_blinkin.get());
  }
}
