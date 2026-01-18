package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class SpindexerConstants {
    private static final int MOTOR_ID = 11;
    private static final String MOTOR_NAME = "SpindexerMotor";
    static final TalonFXSMotor MOTOR = new TalonFXSMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 25;

    private static final DCMotor GEARBOX = DCMotor.getMinion(1);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SPINDEXER_SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(2).per(Units.Seconds),
            Units.Volts.of(8),
            null
    );

    static final Pose3d SPINDEXER_VISUALIZATION_POSE = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 2;
    private static final String SPINDEXER_MECHANISM_NAME = "SpindexerMechanism";
    static final SpeedMechanism2d SPINDEXER_MECHANISM = new SpeedMechanism2d(
            SPINDEXER_MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 0.2;

    static {
        final TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.ExternalFeedback.withSensorToMechanismRatio(GEAR_RATIO);

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0.4 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0012991 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.61844 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.011144 : 0;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 10 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 7 : 0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SPINDEXER_SIMULATION);

        MOTOR.registerSignal(TalonFXSSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSSignal.STATOR_CURRENT, 100);
    }

    public enum SpindexerState {
        FEED_TO_TURRET(1.0),
        AGITATE(0.6),
        STOP(0.0);

        public final double targetVelocityRotationsPerSecond;

        SpindexerState(double targetVelocityRotationsPerSecond) {
            this.targetVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
        }
    }
}