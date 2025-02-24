package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
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
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

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

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.kStickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.kStickDeadband);
        double rawRotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.kStickDeadband);

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
                rotationCommand = rotationLimiter.calculate(angleError);
            } else {
                rotationCommand = 0.0;
            }
        }
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.kMaxSpeed),
                rotationCommand * Constants.Swerve.kMaxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}
