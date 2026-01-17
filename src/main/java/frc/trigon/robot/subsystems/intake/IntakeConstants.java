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
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;


public class IntakeConstants {
    private static final int
            INTAKE_ANGLE_MOTOR_ID = 10,
            INTAKE_ANGLE_ENCODER_ID = 10;
    private static final String
            INTAKE_ANGLE_MOTOR_NAME = "IntakeAngleMotor",
            INTAKE_ANGLE_ENCODER_NAME = "IntakeAngleEncoder";
    static final TalonFXMotor ANGLE_MOTOR = new TalonFXMotor(INTAKE_ANGLE_MOTOR_ID, INTAKE_ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(INTAKE_ANGLE_ENCODER_ID, INTAKE_ANGLE_ENCODER_NAME);

    static final double GEAR_RATIO = 42;
    static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 0,
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 5 : 0,
            MAX_JERK = MAX_ACCELERATION * 10;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double
            LENGTH_METERS = 0.5,
            MASS_KILOGRAMS = 5;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(360);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            LENGTH_METERS,
            MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.2).per(Units.Seconds),
            Units.Volts.of(1.5),
            Units.Second.of(1000)
    );

    static final String MECHANISM_NAME = "IntakeAngleMotor";
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            LENGTH_METERS,
            Color.kLightSeaGreen
    );

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0);
    static final boolean FOC_ENABLED = true;

    static {
        configureIntakeAngleEncoder();
        configureIntakeAngleMotor();
    }

    private static void configureIntakeAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
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
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MAX_JERK;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

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

    public enum AngleMotorState {
        HIGH(Rotation2d.fromDegrees(90)),
        LOW(Rotation2d.fromDegrees(45)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        AngleMotorState(Rotation2d angle) {
            this.targetAngle = angle;
        }
    }
}