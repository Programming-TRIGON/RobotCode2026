package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
                getCurrentEncoderAngle().plus(TurretConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)).plus(TurretConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    void alignToHub() {
        // TODO: add logic
    }

    void alignForDelivery() {
        Rotation2d targetAngle = calculateTargetAngleForDelivery();
        Rotation2d targetAngleAfterLimitCheck = limitAngle(targetAngle);
        setTargetAngle(targetAngleAfterLimitCheck);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.minus(TurretConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET).getRotations()));
    }

    private Rotation2d calculateTargetAngleForDelivery() {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        if (!isRobotInHubYRange())
            return Rotation2d.kZero.minus(currentPosition.getRotation());
        if (currentPosition.getY() < FieldConstants.HUB_Y)
            return calculateTargetAngleToPose(new Translation2d(FieldConstants.ALLIANCE_ZONE_LINE_X / 2, FieldConstants.HUB_LEFTMOST_Y / 2));
        return calculateTargetAngleToPose(new Translation2d(FieldConstants.ALLIANCE_ZONE_LINE_X / 2, FieldConstants.HUB_LEFTMOST_Y * 1.5));
    }

    private Rotation2d calculateTargetAngleToPose(Translation2d targetTranslation) {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d difference = targetTranslation.minus(currentPosition.getTranslation());
        final Rotation2d theta = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        return theta.minus(currentPosition.getRotation());
    }

    private boolean isRobotInHubYRange() {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return currentPosition.getY() < FieldConstants.HUB_LEFTMOST_Y || currentPosition.getY() > FieldConstants.HUB_RIGHTMOST_Y;
    }

    private Rotation2d limitAngle(Rotation2d targetAngle) {
        if (isAngleOutOfRange(targetAngle))
            return targetAngle.getDegrees() > 0 ? TurretConstants.ANGULAR_RANGE_PER_SIDE : TurretConstants.ANGULAR_RANGE_PER_SIDE.unaryMinus();
        if (targetAngle.getDegrees() > TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees())
            return targetAngle.minus(TurretConstants.ANGULAR_RANGE);
        if (targetAngle.getDegrees() < -TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees())
            return targetAngle.minus(TurretConstants.ANGULAR_RANGE);
        return targetAngle;
    }

    private boolean isAngleOutOfRange(Rotation2d angle) {
        final double absoluteAngleDegrees = Math.abs(angle.getDegrees());
        return absoluteAngleDegrees > TurretConstants.ANGULAR_RANGE_PER_SIDE.getDegrees() && absoluteAngleDegrees < 360 - TurretConstants.ANGULAR_RANGE.getDegrees();
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }
}