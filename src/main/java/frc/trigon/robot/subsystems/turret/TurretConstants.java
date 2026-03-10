package frc.trigon.robot.subsystems.turret;

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
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.robot.constants.RobotConstants;

public class TurretConstants {
    private static final int
            MASTER_MOTOR_ID = 14,
            FOLLOWER_MOTOR_ID = 15,
            ENCODER_ID = 14;
    private static final String
            MASTER_MOTOR_NAME = "TurretMasterMotor",
            FOLLOWER_MOTOR_NAME = "TurretFollowerMotor",
            ENCODER_NAME = "TurretEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double
            GEAR_RATIO = (5) * (30 / 20.0) * (34 / 32.0) * (9),
            ENCODER_GEAR_RATIO = 112.20977533;
    private static final double CURRENT_LIMIT_AMPS = 55;
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Aligned;
    static final double RESIST_SWERVE_ROTATION_FEEDFORWARD_GAIN = RobotHardwareStats.isSimulation() ? 0 : 0;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.1;
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final String MECHANISM_NAME = "TurretMechanism";
    private static final Color MECHANISM_COLOR = Color.kMediumPurple;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            MECHANISM_COLOR
    );
    public static final Pose3d TURRET_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.14542, 0.14542, 0.34578),
            new Rotation3d(0, 0, 0)
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(4),
            null
    );

    static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(30),
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-380);
    static final Rotation2d
            NORMAL_TOLERANCE = Rotation2d.fromDegrees(3),
            WIDE_TOLERANCE = Rotation2d.fromDegrees(15);
    static final double ROBOT_VELOCITY_TO_FUTURE_ANGLE_SECONDS = 0.2;
    static final double RESIST_Y_MOVEMENT_FOR_DELIVERY_COEFFICIENT = 10;
    static final Rotation2d SELF_RELATIVE_EJECTION_ANGLE = Rotation2d.fromDegrees(0);
    static final double ROBOT_ROTATION_PREDICTION_TIME_SECONDS = 0.1;
    static final double SLOW_SCAN_FOR_APRILTAGS_VOLTAGE = 1.5;

    static final double TURRET_ANGLE_HISTORY_SIZE_SECONDS = 2;
    static final Pose3d TURRET_ORIGIN_POINT_FOR_CAMERA_CALCULATION = new Pose3d(
            new Translation3d(-0.1454, 0.1454, 0.28423),
            new Rotation3d(0, 0, 0)
    );
    static final Transform3d
            TURRET_TO_RIGHT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(0.14853, -0.06965, 0.2511),
            new Rotation3d(Math.toRadians(180), Math.toRadians(-36), Math.toRadians(-30))
    ),
            TURRET_TO_LEFT_CAMERA_TRANSFORM = new Transform3d(
                    new Translation3d(0.14853, 0.06965, 0.2511),
                    new Rotation3d(Math.toRadians(180), Math.toRadians(-36), Math.toRadians(30))
            );

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureEncoder();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ENCODER_GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO / ENCODER_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.VelocityFilterTimeConstant = 0.01;

//        config.ClosedLoopGeneral.GainSchedKpBehavior = GainSchedKpBehaviorValue.Discontinuous;
//        config.ClosedLoopGeneral.GainSchedErrorThreshold = 0;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 270 : 70;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.6 : 0.5;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.01 : 0.2;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 7.5 : 8.0362;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.0005 : 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot0.GainSchedBehavior = GainSchedBehaviorValue.Inactive;

//        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 100 : 0;
//        config.Slot1.kI = config.Slot0.kI;
//        config.Slot1.kD = 2;
//        config.Slot1.kS = config.Slot0.kS;
//        config.Slot1.kV = 0;
//        config.Slot1.kA = 0;
//        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 1.4 : 1.49270441;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 90 : 10;
        config.MotionMagic.MotionMagicJerk = RobotHardwareStats.isSimulation() ? 0 : config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_ANGLE.getRotations();
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINIMUM_ANGLE.getRotations();

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 250);
        MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 250);
        MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 250);
        MASTER_MOTOR.registerThreadedSignal(TalonFXSignal.POSITION, 250);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;

        FOLLOWER_MOTOR.applyConfiguration(config);

        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);

        final Follower followRequest = new Follower(MASTER_MOTOR.getID(), FOLLOWER_ALIGNMENT_TO_MASTER);
        FOLLOWER_MOTOR.setControl(followRequest);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0.403564453125;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.2;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 250);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 250);
    }
}