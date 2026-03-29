package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class ShooterConstants {
    private static final int
            MASTER_MOTOR_ID = 16,
            FOLLOWER_MOTOR_ID = 17;
    private static final String
            MASTER_MOTOR_NAME = "ShooterMasterMotor",
            FOLLOWER_MOTOR_NAME = "ShooterFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double
            LOWER_WHEEL_GEAR_RATIO = (30.0 / 20.0),
            UPPER_WHEEL_GEAR_RATIO = (30.0 / 20.0) * (54.0 / 30.0);
    public static final double
            LOWER_WHEEL_ROTATIONS_PER_METER = LOWER_WHEEL_GEAR_RATIO / (Math.PI * 4 * 0.0254),
            UPPER_WHEEL_ROTATIONS_PER_METER = UPPER_WHEEL_GEAR_RATIO / (Math.PI * 2 * 0.0254);
    public static final double SYSTEM_ROTATIONS_PER_METER = (((LOWER_WHEEL_ROTATIONS_PER_METER + UPPER_WHEEL_ROTATIONS_PER_METER) / 2) + LOWER_WHEEL_ROTATIONS_PER_METER) / 2;
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Opposed;
    private static final double STATOR_CURRENT_LIMIT_AMPS = 100;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.002;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(GEARBOX, SYSTEM_ROTATIONS_PER_METER, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(7),
            null
    );

    private static final String MECHANISM_NAME = "ShooterMechanism";
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 15;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double EJECTION_VELOCITY_METERS_PER_SECOND = 3;
    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.3;
    static final double WHEEL_SLIPPAGE_COMPENSATION_VELOCITY_MULTIPLIER = RobotHardwareStats.isSimulation() ? 1 : 1;
    static final double
            DELIVERY_VELOCITY_SLOPE = 1,
            DELIVERY_VELOCITY_INTERCEPT_POINT = 2.4;

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0.02 : 0.15;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.012165 : 0.24;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.67692 : 0.93977;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.011184 : 0.041229;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 15 : 12.7690818;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 300 : 291.057266;

        config.Feedback.SensorToMechanismRatio = SYSTEM_ROTATIONS_PER_METER;
        config.Feedback.VelocityFilterTimeConstant = 0.01;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 50);
        MASTER_MOTOR.registerSignal(TalonFXSignal.SUPPLY_CURRENT, 50);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 250);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

        FOLLOWER_MOTOR.applyConfiguration(config);

        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 50);
        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.SUPPLY_CURRENT, 50);
        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 250);

        final Follower followRequest = new Follower(MASTER_MOTOR.getID(), FOLLOWER_ALIGNMENT_TO_MASTER).withUpdateFreqHz(1000);
        FOLLOWER_MOTOR.setControl(followRequest);
    }
}