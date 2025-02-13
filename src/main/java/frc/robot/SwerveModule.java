package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.kDriveKS,
            Constants.Swerve.kDriveKV, Constants.Swerve.kDriveKA);

    /* drive motor control requests */
    @SuppressWarnings("unused")
	private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

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
        // Constants.Swerve.maxSpeed;
        // mDriveMotor.setControl(driveDutyCycle);
        // }
        // else {
        // driveVelocity.Velocity =
        // Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
        // Constants.Swerve.wheelCircumference);
        // driveVelocity.FeedForward =
        // driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        // mDriveMotor.setControl(driveVelocity);
        // }

        // System.out.println("Velocity Before" + driveVelocity.Velocity);
        // System.out.println("FeedForward Before" + driveVelocity.FeedForward);
        driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                Constants.Swerve.kWheelCircumference);
        driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        // driveVelocity.FeedForward = 0;
        mDriveMotor.setControl(driveVelocity);
        // System.out.println("Velocity After" + driveVelocity.Velocity);
        // System.out.println("FeedForward After" + driveVelocity.FeedForward);

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
                Constants.Swerve.wheelCircumference);
      
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(),
                Constants.Swerve.wheelCircumference);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModulePosition(distance, angle);
    }
}