package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class SpindexerConstants {
    private static final int MOTOR_ID = 11;
    private static final String MOTOR_NAME = "SpindexerMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final boolean FOC_ENABLED = true;

    private static final double GEAR_RATIO = 5;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
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
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

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

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum SpindexerState {
        FEED_TURRET(1.0),
        INDEX_BALL(0.6),
        STOP(0.0);

        public final double targetVelocityRotationsPerSecond;

        SpindexerState(double targetVelocityRotationsPerSecond) {
            this.targetVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
        }
    }
}