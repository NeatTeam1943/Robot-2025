package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
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

    m_RightMotor.setControl(new Follower(m_LeftMotor.getDeviceID(), true));
    m_MagnetSwitch = new DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
    m_TopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
    m_BottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
     L1 = Constants.ElevatorConstants.kL1EncoderValue;
     L2 =  Constants.ElevatorConstants.kL2EncoderValue;
     L3 =  Constants.ElevatorConstants.kL3EncoderValue;
     L4 =  Constants.ElevatorConstants.kL4EncoderValue;
   } 
   public boolean MagnetSwitchState(){
    System.out.println("Magnet switch {0}" + m_MagnetSwitch.get());
    return m_MagnetSwitch.get();
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
    double encoderValueTreshHold = Constants.ElevatorConstants.kEncoderValueTreshHold;
    if(m_BottomLimitSwitch.get()){
        ElevatorLevel = 0;
    }
    else if(m_TopLimitSwitch.get()){
        ElevatorLevel = 5;
    }
    else if(MagnetSwitchState){
        
        if(EncValue-encoderValueTreshHold < L1 && L1 < EncValue +encoderValueTreshHold)
            ElevatorLevel = 1;
        else if(EncValue-encoderValueTreshHold < L2 && L2 < EncValue +encoderValueTreshHold)
            ElevatorLevel = 2;
        else if(EncValue-encoderValueTreshHold < L3 && L3 < EncValue +encoderValueTreshHold)
            ElevatorLevel = 3;
        else if(EncValue-encoderValueTreshHold < L4 && L4 < EncValue +encoderValueTreshHold)
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
        System.out.println("Top limit switch is " + m_TopLimitSwitch.get());
        return m_TopLimitSwitch.get(); 
    }


    public boolean ElevatorBottomLimitState(){
        System.out.println("Low limit switch is " + m_BottomLimitSwitch.get());
        return m_BottomLimitSwitch.get();
    }
    
    

   public void MoveElevator(double speed){
    m_LeftMotor.set(speed);
   }


}

    