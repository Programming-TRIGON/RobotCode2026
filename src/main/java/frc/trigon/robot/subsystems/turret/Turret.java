package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem {
    private final TalonFXMotor motor = TurretConstants.MOTOR;
    private final CANcoderEncoder encoder = TurretConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TurretConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(TurretConstants.FOC_ENABLED);

    public Turret() {
        setName("Turret");
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("TurretMotor")
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)))
                .angularPosition(Units.Rotations.of(getCurrentEncoderAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(encoder.getSignal(CANcoderSignal.VELOCITY)));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return TurretConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        encoder.update();
    }

    @Override
    public void updateMechanism() {
        TurretConstants.MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        Logger.recordOutput("Poses/Components/TurretPose", calculateVisualizationPose());
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    void alignToHub() {
        // TODO: add logic
    }

    void alignForDelivery() {
        final Rotation2d targetAngle = calculateTargetAngleForDelivery();
        setTargetAngle(targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        final Rotation2d targetAngleAfterLimitCheck = limitAngle(targetAngle);
        motor.setControl(positionRequest.withPosition(targetAngleAfterLimitCheck.getRotations()));
    }

    private Rotation2d calculateTargetAngleForDelivery() {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        if (currentPosition.getY() < FieldConstants.HUB_Y)
            return calculateTargetAngleToPose(FieldConstants.LEFT_DELIVERY_POSITION.get());
        return calculateTargetAngleToPose(FieldConstants.RIGHT_DELIVERY_POSITION.get());
    }

    private Rotation2d calculateTargetAngleToPose(Translation2d targetTranslation) {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return targetTranslation.minus(currentPosition.getTranslation())
                .getAngle()
                .minus(currentPosition.getRotation());
    }

    private Rotation2d limitAngle(Rotation2d targetAngle) {
        if (isAngleOutOfRange(targetAngle))
            return targetAngle.getDegrees() > 0 ? TurretConstants.ANGULAR_RANGE_PER_SIDE : TurretConstants.ANGULAR_RANGE_PER_SIDE.unaryMinus();
        if (targetAngle.getDegrees() > TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees())
            return targetAngle.minus(TurretConstants.ANGULAR_RANGE);
        if (targetAngle.getDegrees() < -TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees())
            return targetAngle.plus(TurretConstants.ANGULAR_RANGE);
        return targetAngle;
    }

    private boolean isAngleOutOfRange(Rotation2d angle) {
        final double absoluteAngleDegrees = Math.abs(angle.getDegrees());
        return absoluteAngleDegrees > TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees() && absoluteAngleDegrees < 360 - TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees();
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d transform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, getCurrentEncoderAngle().getRadians())
        );
        return TurretConstants.TURRET_VISUALIZATION_ORIGIN_POINT.transformBy(transform);
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }
}