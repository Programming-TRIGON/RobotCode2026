package frc.trigon.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class HoodConstants {
    private static final int
            MOTOR_ID = 18,
            ENCODER_ID = 18;
    private static final String
            MOTOR_NAME = "HoodMotor",
            ENCODER_NAME = "HoodEncoder";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 50;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET_ROTATIONS = 0;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET_ROTATION = RobotHardwareStats.isSimulation() ? 0 : 0 - ANGLE_ENCODER_GRAVITY_OFFSET_ROTATIONS;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double
            HOOD_MASS_KILOGRAMS = 0.7,
            HOOD_LENGTH_METERS = 0.17;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(50),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(87);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            HOOD_LENGTH_METERS,
            HOOD_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    private static final String MECHANISM_NAME = "HoodMechanism";
    private static final Color MECHANISM_COLOR = Color.kYellow;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            HOOD_LENGTH_METERS,
            MECHANISM_COLOR
    );

    static final Pose3d HOOD_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.5).per(Units.Seconds),
            Units.Volts.of(1),
            null
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);
    static final Rotation2d
            REST_ANGLE = Rotation2d.fromDegrees(87),
            DELIVERY_ANGLE = Rotation2d.fromDegrees(50);

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 1 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.039753 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.45 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.065 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 20 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 20 : 0;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET_ROTATIONS;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}