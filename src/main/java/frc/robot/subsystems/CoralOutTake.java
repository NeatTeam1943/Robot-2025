package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralOutTake extends SubsystemBase{
    private TalonFX m_Motor;
    private DigitalInput m_PhotoSwitch;
    public CoralOutTake(){
        m_Motor = new TalonFX(Constants.CoralOutTakeConstants.kMotorPort);
        m_PhotoSwitch = new DigitalInput(Constants.CoralOutTakeConstants.kPhotoSwitchPort);
    }

    public boolean PhotoSwitchMode(){
        boolean Mode = m_PhotoSwitch.get();
        return Mode;
    }

    public void coralOut(double speed){
        m_Motor.set(speed);
    }
}
