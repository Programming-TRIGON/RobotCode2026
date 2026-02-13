package frc.trigon.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.robot.subsystems.turret.TurretConstants;

public class HoodConstants {
    private static final int MOTOR_ID = 18;
    private static final String MOTOR_NAME = "HoodMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 49.36;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double
            HOOD_MASS_KILOGRAMS = 0.7,
            HOOD_LENGTH_METERS = 0.17;
    private static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(87),
            MINIMUM_ANGLE = Rotation2d.fromDegrees(50);
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
            new Translation3d(-0.06144, 0.14542, 0.46867),
            new Rotation3d(0, Math.toRadians(87), 0)
    );
    static final Transform3d TURRET_TO_HOOD_OFFSET = HOOD_VISUALIZATION_ORIGIN_POINT.minus(TurretConstants.TURRET_VISUALIZATION_ORIGIN_POINT);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.4).per(Units.Seconds),
            Units.Volts.of(0.75),
            null
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1);
    static final Rotation2d
            REST_ANGLE = Rotation2d.fromDegrees(87),
            DELIVERY_ANGLE = Rotation2d.fromDegrees(56),
            EJECTION_ANGLE = Rotation2d.fromDegrees(75);
    static final double HOOD_RESET_VOLTAGE = 0.5;
    static final Rotation2d RESET_ANGLE = Rotation2d.fromDegrees(87);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Feedback.VelocityFilterTimeConstant = 0.1;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 100 : 500;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 2 : 1;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.07 : 0.42164;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 2.5 : 4.6448;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.065 : -0.083787;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = -0.07425999999999999;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 20 : 2; // 2.44299674267
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 20 : 13; // 15.3028042
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
}