package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;


public class Algea extends SubsystemBase{

    private TalonFX m_motor;

    public Algea() {
        m_motor = new TalonFX(Constants.AlgeaConstants.motorPort);
    }

}
