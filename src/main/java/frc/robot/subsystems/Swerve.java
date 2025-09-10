package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import java.lang.invoke.VarHandle;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
        public SwerveDriveOdometry swerveOdometry;
        public SwerveModule[] m_SwerveMods;
        public Pigeon2 gyro;
        private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        Constants.SwerveConstans.kModuleTranslations);

        public Swerve() {
                gyro = new Pigeon2(Constants.SwerveConstans.kPigeonID);
                Pigeon2Configuration config = new Pigeon2Configuration();
                config.MountPose.MountPoseYaw = -90; // Adjust if your Pigeon is mounted at an angle
                gyro.getConfigurator().apply(config);
                gyro.setYaw(0);

                m_SwerveMods = new SwerveModule[] {
                                new SwerveModule(0, Constants.SwerveConstans.Mod0.kConstants),
                                new SwerveModule(1, Constants.SwerveConstans.Mod1.kConstants),
                                new SwerveModule(2, Constants.SwerveConstans.Mod2.kConstants),
                                new SwerveModule(3, Constants.SwerveConstans.Mod3.kConstants)
                };

                swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstans.kSwerveKinematics, getGyroYaw(),
                                getModulePositions());

                AutoBuilder.configure(
                                this::getPose,
                                this::setPose,
                                this::getRobotRelativeSpeeds,
                                this::runPureVelocity,
                                new PPHolonomicDriveController(
                                                new PIDConstants(5.0, 0.0, 0.0),
                                                new PIDConstants(17, 0.0, 0.0)),
                                Constants.SwerveConstans.kPPConfig,

                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                this);
        }

        public void 
        drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
                SwerveModuleState[] swerveModuleStates = Constants.SwerveConstans.kSwerveKinematics
                                .toSwerveModuleStates(
                                                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                translation.getX(),
                                                                translation.getY(),
                                                                rotation,
                                                                getHeading())
                                                                : new ChassisSpeeds(
                                                                                translation.getX(),
                                                                                translation.getY(),
                                                                                rotation));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstans.kMaxSpeed);

                for (SwerveModule mod : m_SwerveMods) {
                        if (mod == m_SwerveMods[0]) {
                                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
                                double cahnge = 1;// SmartDashboard.getNumber("DB/Slider 0", 0.9);
                                mod.driveVelocity.Velocity /= cahnge;
                        } else {
                                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
                        }
                }
        }

        /* Used by SwerveControllerCommand in Auto */
        public void setModuleStates(SwerveModuleState[] desiredStates) {

                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstans.kMaxSpeed);

                for (SwerveModule mod : m_SwerveMods) {
                        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
                }
        }

        public SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = new SwerveModuleState[4];
                for (SwerveModule mod : m_SwerveMods) {
                        states[mod.moduleNumber] = mod.getState();
                }
                return states;
        }

        public SwerveModulePosition[] getModulePositions() {
                SwerveModulePosition[] positions = new SwerveModulePosition[4];
                for (SwerveModule mod : m_SwerveMods) {
                        positions[mod.moduleNumber] = mod.getPosition();
                }
                return positions;
        }

        public Pose2d getPose() {
                return swerveOdometry.getPoseMeters();
        }

        public void setPose(Pose2d pose) {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }

        public Rotation2d getHeading() {
                return getPose().getRotation();
        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
                return kinematics.toChassisSpeeds(getModuleStates());
        }

        public void runPureVelocity(ChassisSpeeds speeds) {
                var moduleStates = kinematics.toSwerveModuleStates(speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstans.kMaxSpeed);
                setModuleStates(moduleStates);
        }

        public void setHeading(Rotation2d heading) {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                                new Pose2d(getPose().getTranslation(), heading));
        }

        public void zeroHeading() {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                                new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }

        public Rotation2d getGyroYaw() {
                // Convert the Pigeon's yaw angle to a Rotation2d
                return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
        }

        public void resetModulesToAbsolute() {
                for (SwerveModule mod : m_SwerveMods) {
                        mod.resetToAbsolute();
                }
        }

        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        private final MutVoltage m_appliedVoltage = Volts.mutable(0);
        // Mutable holder for unit-safe linear distance values, persisted to avoid
        // reallocation.
        private final MutDistance m_distance = Meters.mutable(0);
        // Mutable holder for unit-safe linear velocity values, persisted to avoid
        // reallocation.
        private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

        private final Velocity<VoltageUnit> rampRate = Volts.per(Seconds).of(0.3);
        private final Voltage stepVoltage = Volts.of(4);
        private final Time timeout = Second.of(10);

        public SysIdRoutine getDriveSysIdRoutine() {
                SwerveModule mod1 = m_SwerveMods[0];
                SwerveModule mod2 = m_SwerveMods[1];
                SwerveModule mod3 = m_SwerveMods[2];
                SwerveModule mod4 = m_SwerveMods[3];
                SysIdRoutine driveRoutine = new SysIdRoutine(new SysIdRoutine.Config(rampRate, stepVoltage, timeout),
                                new SysIdRoutine.Mechanism((voltage) -> {
                                        mod1.getDriveMotor().setVoltage(voltage.in(Volts));
                                        mod2.getDriveMotor().setVoltage(voltage.in(Volts));
                                        mod3.getDriveMotor().setVoltage(voltage.in(Volts));
                                        mod4.getDriveMotor().setVoltage(voltage.in(Volts));
                                },
                                                log -> {
                                                        log.motor("Drive Motor of moudle number : 1 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod1.getDriveMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod1
                                                                                                        .getDriveMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod1
                                                                                                        .getDriveMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Drive Motor of moudle number : 2 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod2.getDriveMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod2
                                                                                                        .getDriveMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod2
                                                                                                        .getDriveMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Drive Motor of moudle number : 3 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod3.getDriveMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod3
                                                                                                        .getDriveMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod3
                                                                                                        .getDriveMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Drive Motor of moudle number : 4 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod4.getDriveMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod4
                                                                                                        .getDriveMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod4
                                                                                                        .getDriveMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                }, this));
                return driveRoutine;
        }

        public SysIdRoutine getAngleSysIdRoutine() {
                SwerveModule mod1 = m_SwerveMods[0];
                SwerveModule mod2 = m_SwerveMods[1];
                SwerveModule mod3 = m_SwerveMods[2];
                SwerveModule mod4 = m_SwerveMods[3];
                SysIdRoutine angleRoutine = new SysIdRoutine(new SysIdRoutine.Config(rampRate, stepVoltage, timeout),
                                new SysIdRoutine.Mechanism((voltage) -> {
                                        mod1.getAngleMotor().setVoltage(voltage.in(Volts));
                                        mod2.getAngleMotor().setVoltage(voltage.in(Volts));
                                        mod3.getAngleMotor().setVoltage(voltage.in(Volts));
                                        mod4.getAngleMotor().setVoltage(voltage.in(Volts));
                                },
                                                log -> {
                                                        log.motor("Angle Motor of moudle number : 1 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod1.getAngleMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod1
                                                                                                        .getAngleMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod1
                                                                                                        .getAngleMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Angle Motor of moudle number : 2 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod2.getAngleMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod2
                                                                                                        .getAngleMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod2
                                                                                                        .getAngleMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Angle Motor of moudle number : 3 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod3.getAngleMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod3
                                                                                                        .getAngleMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod3
                                                                                                        .getAngleMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                        log.motor("Angle Motor of moudle number : 4 ")
                                                                        .voltage(
                                                                                        m_appliedVoltage.mut_replace(
                                                                                                        mod4.getAngleMotor()
                                                                                                                        .get()
                                                                                                                        * RobotController
                                                                                                                                        .getBatteryVoltage(),
                                                                                                        Volts))
                                                                        .linearPosition(m_distance
                                                                                        .mut_replace(mod4
                                                                                                        .getAngleMotor()
                                                                                                        .getRotorPosition()
                                                                                                        .getValueAsDouble(),
                                                                                                        Meters))
                                                                        .linearVelocity(
                                                                                        m_velocity.mut_replace(mod4
                                                                                                        .getAngleMotor()
                                                                                                        .getVelocity()
                                                                                                        .getValueAsDouble(),
                                                                                                        MetersPerSecond));
                                                }, this));
                return angleRoutine;
        }

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction,
                        SysIdRoutine routine) {
                return routine.quasistatic(direction);
        }

        /**
         * Returns a command that will execute a dynamic test in the given direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction, SysIdRoutine routine) {
                return routine.dynamic(direction);
        }

        @Override
        public void periodic() {
                swerveOdometry.update(getGyroYaw(), getModulePositions());

                for (SwerveModule mod : m_SwerveMods) {
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
                                        mod.getCANcoder().getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",
                                        mod.getPosition().angle.getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                                        mod.getState().speedMetersPerSecond);
                }
        }
}