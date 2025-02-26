package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    // TwinStick R Inputs
    private DoubleSupplier twinStickRotX;
    private DoubleSupplier twinStickRotY;

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean precisionMode = false;
    private static final double PRECISION_SCALE = 0.3; // 30% of normal speed

    public TeleopSwerve(Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup
    // No TwinStick
    ) {
        s_Swerve = swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    // TwinStick
    public TeleopSwerve(Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            DoubleSupplier twinStickRotX,
            DoubleSupplier twinStickRotY) {
        this(swerve, translationSup, strafeSup, rotationSup, robotCentricSup);
        this.twinStickRotX = twinStickRotX;
        this.twinStickRotY = twinStickRotY;
    }

    public void togglePrecisionMode() {
        precisionMode = !precisionMode;
        SmartDashboard.putBoolean("Precision Mode", precisionMode);
    }

    public void setPrecisionMode(boolean enabled) {
        precisionMode = enabled;
        SmartDashboard.putBoolean("Precision Mode", precisionMode);
    }

    public boolean isPrecisionMode() {
        return precisionMode;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.kStickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.kStickDeadband);
        double rawRotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.kStickDeadband);

        translationVal = translationLimiter.calculate(translationVal);
        strafeVal = strafeLimiter.calculate(strafeVal);

        double rotationCommand = rawRotationVal;

        // TwinStick Logic
        if (s_Swerve.isTwinStickMode() && twinStickRotX != null && twinStickRotY != null) {
            double twX = MathUtil.applyDeadband(twinStickRotX.getAsDouble(), Constants.kStickDeadband);
            double twY = MathUtil.applyDeadband(twinStickRotY.getAsDouble(), Constants.kStickDeadband);
            if (Math.hypot(twX, twY) > Constants.kStickDeadband) {
                double targetAngle = Math.atan2(twY, twX);
                double currentAngle = s_Swerve.getGyroYaw().getRadians();
                double angleError = Math.atan2(Math.sin(targetAngle - currentAngle),
                        Math.cos(targetAngle - currentAngle));
                // Apply slew rate limiting to smooth rotation commands
                rotationCommand = rotationLimiter.calculate(angleError);
            } else {
                rotationCommand = 0.0;
            }
        } else {
            rotationCommand = rotationLimiter.calculate(rotationCommand);
        }

        if (precisionMode) {
            translationVal *= PRECISION_SCALE;
            strafeVal *= PRECISION_SCALE;
            rotationCommand *= PRECISION_SCALE;
        }

        double maxLinearSpeed = precisionMode ? Constants.Swerve.kMaxSpeed * PRECISION_SCALE
                : Constants.Swerve.kMaxSpeed;

        double maxAngularSpeed = precisionMode ? Constants.Swerve.kMaxAngularVelocity * PRECISION_SCALE
                : Constants.Swerve.kMaxAngularVelocity;

        SmartDashboard.putNumber("Drive/Translation", translationVal);
        SmartDashboard.putNumber("Drive/Strafe", strafeVal);
        SmartDashboard.putNumber("Drive/Rotation", rotationCommand);
        SmartDashboard.putBoolean("Drive/FieldRelative", !robotCentricSup.getAsBoolean());

        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(maxLinearSpeed),
                rotationCommand * maxAngularSpeed,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}
