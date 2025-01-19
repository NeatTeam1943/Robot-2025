package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase{
    private TalonFX m_Motor;
    private DigitalInput m_PhotoSwitch;
        public CoralIntake(){
        m_PhotoSwitch = new DigitalInput(Constants.CoralIntakeConstants.kPhotoSwitchPort);
        m_Motor = new TalonFX(Constants.CoralIntakeConstants.kMotorPort);
    }
    

    public boolean PhotoSwitchMode(){
        boolean mode = m_PhotoSwitch.get();
        return mode;
    }

    public void coralTransport(double speed){
        m_Motor.set(speed);
    }
    
}
