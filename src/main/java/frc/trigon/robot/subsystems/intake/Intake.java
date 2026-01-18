package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor angleMotor = IntakeConstants.ANGLE_MOTOR;
    private final TalonFXMotor wheelMotor = IntakeConstants.WHEEL_MOTOR;
    private final CANcoderEncoder angleEncoder = IntakeConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IntakeAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));

        log.motor("IntakeWheelMotor")
                .angularPosition(Units.Rotations.of(getWheelPosition().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityRps()))
                .voltage(Units.Volts.of(wheelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.ANGLE_MOTOR_MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        IntakeConstants.WHEEL_MOTOR_MECHANISM.update(
                getCurrentVelocityRps(),
                wheelMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );

        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        angleMotor.setControl(voltageRequest.withOutput(targetVoltage));
        //wheelMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        angleMotor.update();
        angleEncoder.update();
        wheelMotor.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentAngle().getDegrees());
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
        wheelMotor.stopMotor();
    }

    public boolean atTargetAngle() {
        return atAngle(targetAngle);
    }

    public void setTargetVoltage(double targetVoltage) {
        voltageRequest.withOutput(targetVoltage);
    }

    public void AngleMotorSetTargetState(IntakeConstants.AngleMotorState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    public void WheelMotorSetTargetState(IntakeConstants.WheelMotorState targetState) {
        wheelMotor.setControl(voltageRequest.withOutput(targetState.targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        Rotation2d clampedAngle = Rotation2d.fromDegrees(
                MathUtil.clamp(
                        targetAngle.getDegrees(),
                        IntakeConstants.ANGLE_MOTOR_MINIMUM_ANGLE.getDegrees(),
                        IntakeConstants.ANGLE_MOTOR_MAXIMUM_ANGLE.getDegrees()
                )
        );

        this.targetAngle = clampedAngle;
        angleMotor.setControl(
                positionRequest.withPosition(clampedAngle.getRotations())
        );
    }

    boolean atAngle(Rotation2d angle) {
        return Math.abs(
                angle.minus(getCurrentAngle()).getDegrees()
        ) < IntakeConstants.ANGLE_MOTOR_TOLERANCE.getDegrees();
    }

    Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                angleMotor.getSignal(TalonFXSignal.POSITION)
        );
    }

    private double getCurrentVelocityRps() {
        return wheelMotor.getSignal(TalonFXSignal.VELOCITY);
    }

    private Rotation2d getWheelPosition() {
        return Rotation2d.fromRotations(wheelMotor.getSignal(TalonFXSignal.POSITION));
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(0, 0,0 ),
                new Rotation3d(0, -getCurrentAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(intakeTransform);
    }
}