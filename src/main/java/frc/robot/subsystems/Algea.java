package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;


public class Algea extends SubsystemBase{

    private TalonFX m_motor;
    private DigitalInput m_photoSwitch;

    public Algea() {
        m_motor = new TalonFX(Constants.AlgeaConstants.kMotorPort);
        m_photoSwitch = new DigitalInput(Constants.AlgeaConstants.kPhotoSwitchPort);
    }

    public void SetAlgeaSpeed(double speed) {
        m_motor.set(speed);
    }
    public Boolean PhotoSwitchState() {
        return m_photoSwitch.get();
    }
}