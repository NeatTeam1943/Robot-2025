// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedController extends SubsystemBase {
  /** Creates a new LedController. */
  Spark m_blinkin;

  public LedController() {
    m_blinkin = new Spark(Constants.LedConstants.kBlinkinControllerPort);
  }

  public enum BlinkinPattern {
    /*
     * Fixed Palette Pattern
     */
    RanbowRainbowPalette(-0.99),
    RainbowPartyPalette(-0.97),
    RainbowOceanPalette(-0.95),
    RainbowLavaPalette(-0.93),
    RainbowForestPalette(-0.91),
    RainbowWithGlitter(-0.89),
    Confeti(-0.87),
    ShotRed(-0.85),
    ShotBlue(-0.83),
    ShotWhite(-0.81),
    SinelonRainbowPalette(-0.79),
    SinelonPartyPalette(-0.77),
    SinelonOceanPalette(-0.75),
    SinelonLavaPalette(-0.73),
    SinelonForestPalette(-0.71),
    BeatsPerMinuteRainbowPalette(-0.69),
    BeatsPerMinutePartyPalette(-0.67),
    BeatsPerMinuteOceanPalette(-0.65),
    BeatsPerMinuteLavaPalette(-0.63),
    BeatsPerMinuteForestPalette(-0.61),
    FireMeduim(-0.59),
    FireLarge(-0.57),
    TwinklesRainbowPalette(-0.55),
    TwinlesPartyPalette(-0.53),
    TwinklesOceanPalette(-0.51),
    TwinklesLavaPalette(-0.49),
    TwinklesForestPalette(-0.47),
    ColorWavesRainbowPalette(-0.45),
    ColorWavesPartyPalette(-0.43),
    ColorWavesOceanPalette(-0.41),
    ColorWaveLavaPalette(-0.39),
    ColorWavesForestPalette(-0.37),
    LarsonScannerRed(-0.35),
    LarsonScannerGray(-0.33),
    LightChaseRed(-0.31),
    LightChaseBlue(-0.29),
    LightChaseGray(-0.27),
    HeartBeatRed(-0.25),
    HeartBeatBlue(-0.23),
    HeartBeatWhite(-0.21),
    HeartBeatGray(-0.19),
    BreathRed(-0.17),
    BreathBlue(-0.15),
    BREATH_GRAY(-0.13),
    StrobeRed(-0.11),
    StrobeBlue(-0.09),
    StrobeGold(-0.07),
    StrobeWhite(-0.05),
    DontDoThisNeatTeam(0.51),
    /*
     * Solid color
     */
    HotPink(+0.57),
    DrakRed(+0.59),
    ThisIsBestNeatTeam(+0.61),
    RedOrange(+0.63),
    Orange(+0.65),
    Gold(+0.67),
    Yellow(+0.69),
    LawnGreen(+0.71),
    Lime(+0.73),
    Dark_Green(+0.75),
    Green(+0.77),
    BlueGreen(+0.79),
    Aqua(+0.81),
    SkyBlue(+0.83),
    DarkBlue(+0.85),
    Blue(+0.87),
    BlueViolet(+0.89),
    Violet(+0.91),
    White(+0.93),
    Gray(+0.95),
    DarkGray(+0.97),
    Black(+0.99);

    public final double value;

    private BlinkinPattern(double value) {
      this.value = value;
    }
  };

  double m_color = DefualtColor();
  BlinkinPattern m_CurrentPattern;

  public void setLedColor(BlinkinPattern pattern) {
    m_CurrentPattern = pattern;
    m_color = (m_CurrentPattern.value);
  }

  public double DefualtColor() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return BlinkinPattern.DrakRed.value;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return BlinkinPattern.Blue.value;
    } else {
      return BlinkinPattern.Green.value;
    }
  }

  public void setToDefault() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      setLedColor(BlinkinPattern.DrakRed);
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setLedColor(BlinkinPattern.Blue);
    } else {
      setLedColor(BlinkinPattern.Green);
    }
  }

  public void ColorSetter(BlinkinPattern color) {
    m_blinkin.set(color.value);
  }

  @Override
  public void periodic() {
    m_blinkin.set(m_color);
    SmartDashboard.putString("DB/String 1", m_CurrentPattern + "");
  }
}
