package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class TurretConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14,
            ENCODER_ID = 13;
    private static final String
            MASTER_MOTOR_NAME = "TurretMasterMotor",
            FOLLOWER_MOTOR_NAME = "TurretFollowerMotor",
            ENCODER_NAME = "TurretEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 52;
    private static final double CURRENT_LIMIT_AMPS = 100;
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Aligned;
    static final double RESIST_SWERVE_ROTATION_FEEDFORWARD_GAIN = 6.2;

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
            Units.Volts.of(3),
            null
    );

    static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(179.5),
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-179.5),
            TOTAL_ANGULAR_RANGE = MAXIMUM_ANGLE.minus(MINIMUM_ANGLE);
    static final double ROBOT_VELOCITY_TO_FUTURE_ANGLE_SECONDS = 0.2;

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
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.ClosedLoopGeneral.GainSchedKpBehavior = GainSchedKpBehaviorValue.Discontinuous;
        config.ClosedLoopGeneral.GainSchedErrorThreshold = 0.007;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 400 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.02 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot0.GainSchedBehavior = GainSchedBehaviorValue.UseSlot1;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 400 : 0;
        config.Slot1.kI = config.Slot0.kI;
        config.Slot1.kD = 0;
        config.Slot1.kS = config.Slot0.kS;
        config.Slot1.kV = 0;
        config.Slot1.kA = 0;
        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 1.5 : 5;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 90 : 5;
        config.MotionMagic.MotionMagicJerk = 0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
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

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}