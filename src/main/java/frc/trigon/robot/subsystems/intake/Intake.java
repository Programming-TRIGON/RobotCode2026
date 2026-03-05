package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.Phoenix6SignalThread;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.misc.MechanismCameraTransformCalculator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor angleMotor = IntakeConstants.ANGLE_MOTOR;
    private final TalonFXMotor masterIntakeMotor = IntakeConstants.MASTER_INTAKE_MOTOR;
    private final TalonFXMotor followerIntakeMotor = IntakeConstants.FOLLOWER_INTAKE_MOTOR;
    private final CANcoderEncoder angleEncoder = IntakeConstants.ANGLE_ENCODER;
    private final MechanismCameraTransformCalculator intakeCameraTransformCalculator = new MechanismCameraTransformCalculator(
            IntakeConstants.INTAKE_ANGLE_HISTORY_SIZE_SECONDS,
            IntakeConstants.INTAKE_ORIGIN_POINT_FOR_CAMERA_CALCULATION,
            this::getCurrentArmAngle,
            false
    );
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private double[] latestThreadedAngles = new double[0];
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IntakeAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentArmAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.ANGLE_MECHANISM.update(
                getCurrentArmAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        IntakeConstants.INTAKE_MOTOR_MECHANISM.update(
                getCurrentIntakeVoltage()
        );

        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        angleMotor.setControl(voltageRequest.withOutput(targetVoltage));
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
        masterIntakeMotor.update();
        followerIntakeMotor.update();
        angleEncoder.update();
        Logger.recordOutput("Intake/CurrentPositionDegrees", getCurrentArmAngle().getDegrees());
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
        masterIntakeMotor.stopMotor();
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
    }

    public Transform3d calculateIntakeCameraTransformAtTime(double timestampSeconds) {
        return intakeCameraTransformCalculator.calculateRobotToCameraAtTime(timestampSeconds, IntakeConstants.ORIGIN_TO_CAMERA_TRANSFORM);
    }

    public void updateLatestThreadedPositions() { // TODO: This function and logic are ugly. Find a better way to do this, perhaps in the Phoenix6SignalThread class itself.
        angleMotor.update();
        latestThreadedAngles = angleMotor.getThreadedSignal(TalonFXSignal.POSITION);
    }

    public void updateCameraTransforms() {
        intakeCameraTransformCalculator.update(
                latestThreadedAngles,
                Phoenix6SignalThread.getInstance().getLatestTimestamps(),
                angleMotor.getSignal(TalonFXSignal.VELOCITY)
        );
    }

    public boolean atState(IntakeConstants.IntakeState targetState) {
        return atAngle(targetState.targetAngle) && targetState == this.targetState;
    }

    public boolean atAngle(Rotation2d targetAngle) {
        return Math.abs(
                targetAngle.minus(getCurrentArmAngle()).getDegrees()
        ) < IntakeConstants.ANGLE_MOTOR_TOLERANCE.getDegrees();
    }

    public boolean atTargetState() {
        return atAngle(this.targetState.targetAngle);
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetAngle, targetState.targetVoltage);
    }

    void setTargetState(Rotation2d targetAngle, double targetVoltage) {
        setTargetAngle(targetAngle);
        setTargetIntakeVoltage(targetVoltage);
    }

    private void setTargetAngle(Rotation2d targetAngle) {
//        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private void setTargetIntakeVoltage(double targetVoltage) {
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(targetVoltage);
        masterIntakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private double getCurrentIntakeVoltage() {
        return masterIntakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -getCurrentArmAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }

    private Rotation2d getCurrentArmAngle() {
        return new Rotation2d();
//        return Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.POSITION));
    }
}