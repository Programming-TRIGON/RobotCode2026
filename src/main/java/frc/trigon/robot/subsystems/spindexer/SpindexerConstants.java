package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
    private static final int SPINDEXER_ID = 11;
    private static final String SPINDEXER_NAME = "Spindexer";
    static final TalonFXMotor SPINDEXER_MOTOR = new TalonFXMotor(SPINDEXER_ID, SPINDEXER_NAME);

    static final double GEAR_RATIO = 5;
    static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 10 : 0,
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 7 : 0;

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
            Units.Second.of(1000)
    );

    public static final Pose3d SPINDEXER_VISUALIZATION_POSE = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );


    static final double MAX_DISPLAYABLE_VELOCITY = 10;
    static final String SPINDEXER_MECHANISM_NAME = "Spindexer";
    static final double DEADBAND = 0.02;
    static final SpeedMechanism2d SPINDEXER_MECHANISM = new SpeedMechanism2d(
            SPINDEXER_MECHANISM_NAME,
            MAX_DISPLAYABLE_VELOCITY,
            DEADBAND
    );

    static final double VELOCITY_TOLERANCE = 2;
    static final boolean FOC_ENABLED = true;

    static {
        configureMotor();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0.0015934 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0012991 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.61844 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.011144 : 0;

        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        SPINDEXER_MOTOR.applyConfiguration(config);
        SPINDEXER_MOTOR.setPhysicsSimulation(SPINDEXER_SIMULATION);

        SPINDEXER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        SPINDEXER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        SPINDEXER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        SPINDEXER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        SPINDEXER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum SpindexerState {
        SPIN_CLOCKWISE(10),
        SPIN_COUNTERCLOCKWISE(-10),
        STOP(0);


        public final double targetVelocity;

        SpindexerState(double targetVelocity) {
            this.targetVelocity = targetVelocity;
        }
    }
}
