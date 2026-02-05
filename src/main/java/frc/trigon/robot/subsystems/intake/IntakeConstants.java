package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.*;
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
import frc.trigon.robot.constants.RobotConstants;

public class IntakeConstants {
    private static final int
            MASTER_INTAKE_MOTOR_ID = 9,
            FOLLOWER_INTAKE_MOTOR_ID = 10,
            ANGLE_MOTOR_ID = 11,
            ANGLE_ENCODER_ID = 11;
    private static final String
            MASTER_INTAKE_MOTOR_NAME = "MasterIntakeMotor",
            FOLLOWER_INTAKE_MOTOR_NAME = "FollowerIntakeMotor",
            ANGLE_MOTOR_NAME = "IntakeAngleMotor",
            ANGLE_ENCODER_NAME = "IntakeAngleEncoder";
    static final TalonFXMotor
            MASTER_INTAKE_MOTOR = new TalonFXMotor(MASTER_INTAKE_MOTOR_ID, MASTER_INTAKE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_INTAKE_MOTOR = new TalonFXMotor(FOLLOWER_INTAKE_MOTOR_ID, FOLLOWER_INTAKE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Aligned;
    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 40,
            INTAKE_MOTOR_GEAR_RATIO = 2.6;
    static final boolean FOC_ENABLED = true;

    private static final int
            ANGLE_MOTOR_AMOUNT = 1,
            INTAKE_MOTOR_AMOUNT = 2;
    private static final DCMotor
            ANGLE_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT),
            INTAKE_GEARBOX = DCMotor.getFalcon500Foc(INTAKE_MOTOR_AMOUNT);
    private static final double
            INTAKE_LENGTH_METERS = 0.23,
            INTAKE_MASS_KILOGRAMS = 3;
    static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(120);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    private static final double WHEEL_MOTOR_MOMENT_OF_INERTIA = 0.003;
    static final SingleJointedArmSimulation INTAKE_ANGLE_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );
    static final SimpleMotorSimulation INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            WHEEL_MOTOR_MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.2).per(Units.Seconds),
            Units.Volts.of(1.5),
            Units.Second.of(1000)
    );

    private static final String
            ANGLE_MOTOR_MECHANISM_NAME = "IntakeAngleMotorMechanism",
            INTAKE_MOTOR_MECHANISM_NAME = "IntakeWheelMotorMechanism";
    private static final Color ANGLE_MOTOR_MECHANISM_COLOR = Color.kOrange;
    private static final double INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE = 12;
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            ANGLE_MOTOR_MECHANISM_NAME,
            INTAKE_LENGTH_METERS,
            ANGLE_MOTOR_MECHANISM_COLOR
    );
    static final SpeedMechanism2d INTAKE_MOTOR_MECHANISM = new SpeedMechanism2d(
            INTAKE_MOTOR_MECHANISM_NAME,
            INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE
    );
    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0.14985, 0, 0.13525),
            new Rotation3d(0, 0, 0)
    );

    static final Rotation2d ANGLE_MOTOR_TOLERANCE = Rotation2d.fromDegrees(2);
    static final double INTAKE_ANGLE_HISTORY_SIZE_SECONDS = 2;
    static final Pose3d INTAKE_ORIGIN_POINT_FOR_CAMERA_CALCULATION = new Pose3d(
            new Translation3d(0.14985, 0, 0.13525),
            new Rotation3d(0, 0, 0)
    );
    static final Transform3d ORIGIN_TO_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(0.19, -0.3, 0.3),
            new Rotation3d(0, Math.toRadians(20), Math.toRadians(30))
    );

    static {
        configureAngleMotor();
        configureMasterIntakeMotor();
        configureFollowerIntakeMotor();
        configureAngleEncoder();
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 30 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.048463 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 5 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.066678 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 5 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 5 : 0;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(INTAKE_ANGLE_SIMULATION);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 250);
        ANGLE_MOTOR.registerThreadedSignal(TalonFXSignal.POSITION, 250);
    }

    private static TalonFXConfiguration createIntakeMotorConfig() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = INTAKE_MOTOR_GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;

        return config;
    }

    private static void configureMasterIntakeMotor() {
        MASTER_INTAKE_MOTOR.applyConfiguration(createIntakeMotorConfig());
        MASTER_INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        MASTER_INTAKE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_INTAKE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerIntakeMotor() {
        FOLLOWER_INTAKE_MOTOR.applyConfiguration(createIntakeMotorConfig());
        FOLLOWER_INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        final Follower followRequest = new Follower(MASTER_INTAKE_MOTOR.getID(), FOLLOWER_ALIGNMENT_TO_MASTER);
        FOLLOWER_INTAKE_MOTOR.setControl(followRequest);

        FOLLOWER_INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        FOLLOWER_INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 250);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 250);
    }

    public enum IntakeState {
        REST(Rotation2d.fromDegrees(90), 0),
        PREPARE_TO_INTAKE(Rotation2d.fromDegrees(0), 0),
        INTAKE(Rotation2d.fromDegrees(0), 6),
        EJECT(Rotation2d.fromDegrees(0), -6);

        public final Rotation2d targetAngle;
        public final double targetVoltage;

        IntakeState(Rotation2d targetAngle, double targetVoltage) {
            this.targetAngle = targetAngle;
            this.targetVoltage = targetVoltage;
        }
    }
}