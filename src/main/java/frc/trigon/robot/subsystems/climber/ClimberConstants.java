package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.ElevatorSimulation;
import frc.trigon.lib.utilities.mechanisms.ElevatorMechanism2d;

public class ClimberConstants {
    private static final int MOTOR_ID = 18;
    private static final String MOTOR_NAME = "ClimberMasterMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    private static final double GEAR_RATIO = 280;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 80 : 20,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 80 : 50;
    static final boolean FOC_ENABLED = true;

    static final double MINIMUM_CLIMBER_HEIGHT_METERS = 0.73;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double
            CLIMBER_MASS_KILOGRAMS = 60,
            DRUM_RADIUS_METERS = 0.04,
            MAXIMUM_CLIMBER_HEIGHT_METERS = 1.8;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
            GEAR_RATIO,
            CLIMBER_MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            MINIMUM_CLIMBER_HEIGHT_METERS,
            MAXIMUM_CLIMBER_HEIGHT_METERS,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.25).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    public static final Pose3d CLIMBER_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    private static final String MECHANISM_NAME = "ClimberMechanism";
    private static final Color MECHANISM_COLOR = Color.kYellow;
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_CLIMBER_HEIGHT_METERS,
            MINIMUM_CLIMBER_HEIGHT_METERS,
            MECHANISM_COLOR
    );

    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double POSITION_TOLERANCE_METERS = 0.07;
    static final double CLIMBER_RESET_VOLTAGE = -0.5;
    static final int CLIMBER_WEIGHT_SLOT = 0;
    static final int ROBOT_WEIGHT_SLOT = 1;

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0087929 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 2.7126 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.052127 : 0;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kG = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
    }

    public enum ClimberState {
        REST(0, 0, 0.7, 0.7),
        CLIMB_L1(0.5, 0, 1, 0.5);

        public final double targetExtendedPositionMeters;
        public final double targetRetractedPositionMeters;
        final double extendedSpeedScalar;
        final double retractedSpeedScalar;

        ClimberState(double targetExtendedPositionMeters, double targetRetractedPositionMeters, double extendedSpeedScalar, double retractedSpeedScalar) {
            this.targetExtendedPositionMeters = targetExtendedPositionMeters;
            this.targetRetractedPositionMeters = targetRetractedPositionMeters;
            this.extendedSpeedScalar = extendedSpeedScalar;
            this.retractedSpeedScalar = retractedSpeedScalar;
        }
    }
}