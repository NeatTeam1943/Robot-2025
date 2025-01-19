package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;


public class Algea extends SubsystemBase{

    private TalonFX m_motor;
    private DigitalInput m_photoSwitch;

    public Algea() {
        m_motor = new TalonFX(Constants.AlgeaConstants.motorPort);
        m_photoSwitch = new DigitalInput(Constants.AlgeaConstants.photoSwitchPort);
    }

    public void move(double speed) {
        m_motor.set(speed);
    }

    public Boolean isAlgeaIn() {
        return m_photoSwitch.get();
    }



}
