package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class CoralOutTake {
    private TalonFX m_Motor;
    private DigitalInput m_PhotoSwitch;
    public CoralOutTake(){
        m_Motor = new TalonFX(Constants.CoralOutTakeConstants.motorPort);
        m_PhotoSwitch = new DigitalInput(Constants.CoralOutTakeConstants.photoSwitchPort);
    }

    public boolean PhotoSwitchMode(){
        boolean Mode = m_PhotoSwitch.get();
        return Mode;
    }

    public void coralOut(double speed){
        m_Motor.set(speed);
    }
}
