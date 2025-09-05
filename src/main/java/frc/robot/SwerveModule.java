package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstans;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstans.kDriveKS,
            SwerveConstans.kDriveKV, SwerveConstans.kDriveKA);

    /* drive motor control requests */
    @SuppressWarnings("unused")
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    public final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        // if(isOpenLoop){
        // driveDutyCycle.Output = desiredState.speedMetersPerSecond /
        // SwerveConstans.maxSpeed;
        // mDriveMotor.setControl(driveDutyCycle);
        // }
        // else {
        // driveVelocity.Velocity =
        // Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
        // SwerveConstans.wheelCircumference);
        // driveVelocity.FeedForward =
        // driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        // mDriveMotor.setControl(driveVelocity);
        // }

        driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                SwerveConstans.kWheelCircumference);
        driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        // driveVelocity.FeedForward = 0;
        mDriveMotor.setControl(driveVelocity);

    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(),
                SwerveConstans.kWheelCircumference);

        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(),
                SwerveConstans.kWheelCircumference);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModulePosition(distance, angle);
    }

    // SysId Methods
    public void setDriveVoltage(double voltage) {
        mDriveMotor.setVoltage(voltage);
    }

    public void setAngleVoltage(double voltage) {
        mAngleMotor.setVoltage(voltage);
    }

    public double getAngleVelocity() {
        return mAngleMotor.getVelocity().getValueAsDouble();
    }

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);

    private final MutDistance m_Distance = Meters.mutable(0);

    private final MutLinearVelocity m_Velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage ->{
                    mDriverMotor.setDriveVoltage(voltage);
                    mAngleMotor.setAngleVoltage(voltage);
                }, 
                
                log -> {
                
                    log.motor("Motor num1").voltage(
                        m_appliedVoltage.mut_replace(mDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_Distance.mut_replace(mDriveMotor.getRotorPosition().getValueAsDouble()))
                        .linerVelocity(m_Velocity.mut_replace(mDriveMotor.getVelocity().getValueAsDouble()));
                        
                }
                
            this));
}
