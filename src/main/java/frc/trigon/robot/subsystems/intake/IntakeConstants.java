package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;


public class IntakeConstants {
    private static final int
            INTAKE_ANGLE_MOTOR_ID = 10,
            INTAKE_ANGLE_ENCODER_ID = 10,
            INTAKE_WHEEL_MOTOR_ID = 9,
            INTAKE_WHEEL_ENCODER_ID = 9;
    private static final String
            INTAKE_ANGLE_MOTOR_NAME = "IntakeAngleMotor",
            INTAKE_ANGLE_ENCODER_NAME = "IntakeAngleEncoder",
            INTAKE_WHEEL_MOTOR_NAME = "IntakeWheelMotor",
            INTAKE_WHEEL_ENCODER_NAME = "IntakeWheelEncoder";
    static final TalonFXMotor ANGLE_MOTOR = new TalonFXMotor(INTAKE_ANGLE_MOTOR_ID, INTAKE_ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(INTAKE_ANGLE_ENCODER_ID, INTAKE_ANGLE_ENCODER_NAME);
    static final TalonFXMotor WHEEL_MOTOR = new TalonFXMotor(INTAKE_WHEEL_MOTOR_ID, INTAKE_WHEEL_MOTOR_NAME);
    static final CANcoderEncoder WHEEL_ENCODER = new CANcoderEncoder(INTAKE_WHEEL_ENCODER_ID, INTAKE_WHEEL_ENCODER_NAME);

    static final double ANGLE_MOTOR_GEAR_RATIO = 42;
    static final double WHEEL_MOTOR_GEAR_RATIO = 5;
    static final double
            ANGLE_MOTOR_MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 0,
            ANGLE_MOTOR_MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 5 : 0,
            ANGLE_MOTOR_MAX_JERK = ANGLE_MOTOR_MAX_ACCELERATION * 10,
            WHEEL_MOTOR_MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 0,
            WHEEL_MOTOR_MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 5: 0,
            WHEEL_MOTOR_MAX_JERK = WHEEL_MOTOR_MAX_ACCELERATION * 10;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(2);
    private static final double
            ANGLE_MOTOR_LENGTH_METERS = 0.5,
            ANGLE_MOTOR_MASS_KILOGRAMS = 5;

    private static final Rotation2d
            ANGLE_MOTOR_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            ANGLE_MOTOR_MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    private static final double WHEEL_MOTOR_MOMENT_OF_INERTIA = 0.003;
    static final SingleJointedArmSimulation INTAKE_ANGLE_SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            ANGLE_MOTOR_LENGTH_METERS,
            ANGLE_MOTOR_MASS_KILOGRAMS,
            ANGLE_MOTOR_MINIMUM_ANGLE,
            ANGLE_MOTOR_MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );

    static final SimpleMotorSimulation INTAKE_WHEEL_SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            WHEEL_MOTOR_GEAR_RATIO,
            WHEEL_MOTOR_MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.2).per(Units.Seconds),
            Units.Volts.of(1.5),
            Units.Second.of(1000)
    );

    static final String
            INTAKE_ANGLE_MOTOR_MECHANISM_NAME = "IntakeAngleMotor",
            INTAKE_WHEEL_MOTOR_MECHANISM_NAME = "IntakeWheelMotor";
    static final SingleJointedArmMechanism2d INTAKE_ANGLE_MOTOR_MECHANISM = new SingleJointedArmMechanism2d(
            INTAKE_ANGLE_MOTOR_MECHANISM_NAME,
            ANGLE_MOTOR_LENGTH_METERS,
            Color.kLightSeaGreen
    );

    static final SpeedMechanism2d INTAKE_WHEEL_MOTOR_MECHANISM = new SpeedMechanism2d(
            INTAKE_WHEEL_MOTOR_NAME,
            WHEEL_MOTOR_MAX_VELOCITY
    );

    static final Rotation2d INTAKE_ANGLE_MOTOR_TOLERANCE = Rotation2d.fromDegrees(0);
    static final Rotation2d INTAKE_WHEEL_MOTOR_TOLERANCE = Rotation2d.fromDegrees(0);
    static final boolean FOC_ENABLED = true;

    static {
        configureIntakeAngleEncoder();
        configureIntakeAngleMotor();
        configureIntakeWheel();
        //configureIntakeWheelEncoder();
    }

    private static void configureIntakeAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 3 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.5 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.080163 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 5.4 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.10494 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.33154 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_MOTOR_MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ANGLE_MOTOR_MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ANGLE_MOTOR_MAX_JERK;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(INTAKE_ANGLE_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureIntakeAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void  configureIntakeWheel() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = WHEEL_MOTOR_GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 10 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 7 : 0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        WHEEL_MOTOR.applyConfiguration(config);
        WHEEL_MOTOR.setPhysicsSimulation(INTAKE_WHEEL_SIMULATION);

        WHEEL_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        WHEEL_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        WHEEL_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        WHEEL_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        WHEEL_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

/*
    private static void configureIntakeWheelEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        WHEEL_ENCODER.applyConfiguration(config);
        WHEEL_ENCODER.setSimulationInputsFromTalonFX(WHEEL_MOTOR);

        WHEEL_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        WHEEL_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
*/

    public enum AngleMotorState {
        HIGH(Rotation2d.fromDegrees(90)),
        LOW(Rotation2d.fromDegrees(45)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        AngleMotorState(Rotation2d angle) {
            this.targetAngle = angle;
        }
    }

    public enum WheelMotorState {
        COLLECT(6),
        REST(0);

        public final double targetVoltage;

        WheelMotorState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}