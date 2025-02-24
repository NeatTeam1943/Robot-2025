package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
    private SparkMax m_Motor;
    private DigitalInput m_PhotoSwitch;

    public Coral() {
        m_Motor = new SparkMax(Constants.CoralConstants.kMotorPort, MotorType.kBrushless);
        m_PhotoSwitch = new DigitalInput(Constants.CoralConstants.kPhotoSwitchPort);
    }

    public boolean PhotoSwitchMode() {
        return m_PhotoSwitch.get();
    }

    public void moveCoral(double speed) {
        m_Motor.set(speed);
        SmartDashboard.putString("DB/String 5", String.valueOf(m_Motor.get()));
    }
}
