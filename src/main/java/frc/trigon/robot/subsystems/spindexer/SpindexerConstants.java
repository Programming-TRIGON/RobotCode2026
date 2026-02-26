package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;
import frc.trigon.robot.constants.RobotConstants;

public class SpindexerConstants {
    private static final int MOTOR_ID = 12;
    private static final String MOTOR_NAME = "SpindexerMotor";
    static final TalonFXSMotor MOTOR = new TalonFXSMotor(MOTOR_ID, MOTOR_NAME, RobotConstants.CANIVORE_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 1 / ((1 / 9.0) * (Math.PI * edu.wpi.first.math.util.Units.inchesToMeters(6)));

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getMinion(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.001;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(4),
            null
    );

    static final Pose3d VISUALIZATION_ORIGIN_POSE = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final String MECHANISM_NAME = "SpindexerMechanism";
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.2;
    static final double SIMULATION_SLIPPAGE_COMPENSATION_MULTIPLIER = 1 / 4.0;
    static final double LOADING_SPEED_RELATIVE_TO_SHOOTING_COEFFICIENT = 1;

    static {
        final TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.ExternalFeedback.withSensorToMechanismRatio(GEAR_RATIO);

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0.005 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0069036 : 0.10229;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.83307599933 : 1.7821;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.046475 : 0.049243;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 10 : 6.73362886;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 60 : 243.689458;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSSignal.STATOR_CURRENT, 100);
    }

    public enum SpindexerState {
        LOAD_TO_TURRET(10),
        STOP(0);

        public final double targetVelocityMetersPerSecond;

        SpindexerState(double targetVelocityMetersPerSecond) {
            this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        }
    }
}