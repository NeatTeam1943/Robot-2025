package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX m_LeftMotor;
    private TalonFX m_RightMotor;
    private double m_leftEncoder;
    private double m_rightEncoder;
    public DigitalInput m_MagnetSwitch;
    private DigitalInput m_TopLimitSwitch;
    private DigitalInput m_BottomLimitSwitch;
    private double L1;
    private double L2;
    private double L3;
    private double L4;
    
   public Elevator(){
    m_LeftMotor = new TalonFX(Constants.ElevatorConstants.kLeftMotorPort);
    m_RightMotor = new TalonFX(Constants.ElevatorConstants.kRightMotorPort);
    m_MagnetSwitch = new DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
    m_TopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
    m_BottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
     L1 = Constants.ElevatorConstants.kL1EncoderValue;
     L2 =  Constants.ElevatorConstants.kL2EncoderValue;
     L3 =  Constants.ElevatorConstants.kL3EncoderValue;
     L4 =  Constants.ElevatorConstants.kL4EncoderValue;
   } 
   public boolean MagnetSwitchState(){
    boolean bool = m_MagnetSwitch.get();
    return bool;
}

    public double EncoderValue(){
        m_leftEncoder = m_LeftMotor.getRotorPosition().getValueAsDouble();
        m_rightEncoder = m_RightMotor.getRotorPosition().getValueAsDouble();
        return (m_leftEncoder+m_rightEncoder)/2;
    }

   public int ElevatorLevel(){ 
    int ElevatorLevel = -1;
    double EncValue = EncoderValue();
    boolean MagnetSwitchState = MagnetSwitchState();
    if(m_BottomLimitSwitch.get()){
        ElevatorLevel = 0;
    }
    else if(m_TopLimitSwitch.get()){
        ElevatorLevel = 5;
    }
    else if(MagnetSwitchState){
        
        if(EncValue-0.01 < L1 && L1 < EncValue +0.01)
            ElevatorLevel = 1;
        else if(EncValue-0.01 < L2 && L2 < EncValue +0.01)
            ElevatorLevel = 2;
        else if(EncValue-0.01 < L3 && L3 < EncValue +0.01)
            ElevatorLevel = 3;
        else if(EncValue-0.01 < L4 && L4 < EncValue +0.01)
            ElevatorLevel = 4;
        else 
            ElevatorLevel = -1;
        }
    else{
        ElevatorLevel = -1;

    }
    return ElevatorLevel;
    }
        

   
            
    public boolean ElevatorTopLimitState(){
        boolean bool  = m_TopLimitSwitch.get();
        return bool; 
    }


    public boolean ElevatorBottomLimitState(){
        boolean bool = m_BottomLimitSwitch.get();
        return bool;
    }
    
    

   public void MoveElevator(int speed){
    m_LeftMotor.set(speed);
    m_RightMotor.set(speed);
   }


}

    