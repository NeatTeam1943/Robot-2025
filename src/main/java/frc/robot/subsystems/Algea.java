// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

/**
 * Add your docs here.
 */
public class Algea extends SubsystemBase {

    private TalonFX m_Motor;

    public Algea() {
        m_Motor = new TalonFX(Constants.AlgeaConstans.kMotorPort);
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public double GetEncoderValue() {
        return m_Motor.getRotorPosition().getValueAsDouble();
    }

    public int getMoveDirection(int requestedLevel) {
        return (int) (GetEncoderValue()
                - lvlEncoderValue(requestedLevel) / Math.abs(GetEncoderValue() - lvlEncoderValue(requestedLevel)));
    }

    public double lvlEncoderValue(int level) {
        switch (level) {
            case 0:
                return Constants.AlgeaConstans.kClosedEncoderValue;
            case 1:
                return Constants.AlgeaConstans.kLowerAlgaeEncoderValue;
            case 2:
                return Constants.AlgeaConstans.kUpperALgeaEncoderValue;
        }
        return 0;
    }

    public double GetStallSpeed() {
        if (Math.abs(Constants.AlgeaConstans.kClosedEncoderValue
                - GetEncoderValue()) > Constants.AlgeaConstans.kEncoderValueTreshHold) {
            return Constants.AlgeaConstans.kClosedStallSpeed;
        }
        if (Math.abs(Constants.AlgeaConstans.kLowerAlgaeEncoderValue
                - GetEncoderValue()) > Constants.AlgeaConstans.kEncoderValueTreshHold) {
            return Constants.AlgeaConstans.kLowerStallSpeed;
        }
        if (Math.abs(Constants.AlgeaConstans.kUpperALgeaEncoderValue
                - GetEncoderValue()) > Constants.AlgeaConstans.kEncoderValueTreshHold) {
            return Constants.AlgeaConstans.kUpperstallSpeed;
        }
        return 0;
    }

    public void MoveAlgeaAxis(double speed) {
        m_Motor.set(speed);
    }

}
