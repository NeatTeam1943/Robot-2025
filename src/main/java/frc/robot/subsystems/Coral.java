package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
    private SparkMax m_Motor;
    private DigitalInput m_OutTakePhotoSwitch;
    private DigitalInput m_IntakePhotoSwitch;

    public Coral() {
        m_Motor = new SparkMax(Constants.CoralConstants.kMotorPort, MotorType.kBrushless);
        m_OutTakePhotoSwitch = new DigitalInput(Constants.CoralConstants.kPhotoSwitchPort);
        m_IntakePhotoSwitch = new DigitalInput(Constants.CoralConstants.kIntakePhotoSwitchPort);
    }

    public boolean PhotoSwitchMode() {
        return m_OutTakePhotoSwitch.get();
    }

    public boolean IntakePhotoSwitchMode() {
        return m_IntakePhotoSwitch.get();
    }

    public void moveCoral(double speed) {
        m_Motor.set(speed);

        SmartDashboard.putString("DB/String 5", m_Motor.get() + "");
    }

    public void printState() {
        SmartDashboard.putBoolean("DB/LED 1", IntakePhotoSwitchMode());
        SmartDashboard.putBoolean("DB/LED 2", PhotoSwitchMode());
    }
}
