package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;
import frc.trigon.robot.constants.RobotConstants;

public class LoaderConstants {
    private static final int MOTOR_ID = 13;
    private static final String MOTOR_NAME = "LoaderMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME, RobotConstants.CANIVORE_NAME);

    static final boolean FOC_ENABLED = true;

    private static final double
            GEAR_RATIO = 1,
            ROTATIONS_PER_METER = 1 / ((1 / GEAR_RATIO) * (Math.PI * edu.wpi.first.math.util.Units.inchesToMeters(2)));

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            ROTATIONS_PER_METER,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(4),
            null
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 8;
    private static final String LOADER_MECHANISM_NAME = "LoaderMechanism";
    static final SpeedMechanism2d LOADER_MECHANISM = new SpeedMechanism2d(
            LOADER_MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.1;
    static final double LOADING_SPEED_RELATIVE_TO_SHOOTING_COEFFICIENT = 0.68;

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = ROTATIONS_PER_METER;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0.075402 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.00071285 : 0.38816;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.915306001 : 0.7941;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.029458 : 0.02032;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 10 : 15.1114469;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 60 : 709.094132;

        config.CurrentLimits.StatorCurrentLimit = 90;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 50);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 50);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 50);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 50);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 50);
    }

    public enum LoaderState {
        LOAD_FOR_DELIVERY(6),
        LOAD_FOR_EJECT(4),
        UNJAM(-4),
        STOP(0);

        public final double targetVelocityMetersPerSecond;

        LoaderState(double targetVelocityMetersPerSecond) {
            this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        }
    }
}