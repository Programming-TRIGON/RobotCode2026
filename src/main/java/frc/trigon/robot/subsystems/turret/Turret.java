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

import java.util.ArrayList;

public class Turret extends MotorSubsystem {
    private final TalonFXMotor motor = TurretConstants.MASTER_MOTOR;
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
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
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

        Logger.recordOutput("Turret/CurrentAngleDegrees", getCurrentEncoderAngle().getDegrees());
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

    void alignToClosestAprilTag() {
        // TODO: add logic
    }

    void alignForDelivery() {
        final Rotation2d targetAngle = calculateTargetAngleForDelivery();
        setTargetSelfRelativeAngleAngle(targetAngle);
    }

    void setTargetFieldRelativeAngle(Rotation2d targetAngle) {
        final Rotation2d targetRobotRelativeAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getDegrees());
        final Rotation2d targetTurretRelativeAngle = Rotation2d.fromDegrees(targetRobotRelativeAngle.getDegrees() + 180);
        setTargetSelfRelativeAngleAngle(targetTurretRelativeAngle);
    }

    void setTargetSelfRelativeAngleAngle(Rotation2d targetAngle) {
        final Rotation2d targetAngleAfterLimitCheck = limitAngle(targetAngle);
        motor.setControl(positionRequest.withPosition(targetAngleAfterLimitCheck.getRotations()));
    }

    private Rotation2d calculateTargetAngleForDelivery() {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        if (currentPosition.getTranslation().getDistance(FieldConstants.LEFT_DELIVERY_POSITION.get()) < currentPosition.getTranslation().getDistance(FieldConstants.RIGHT_DELIVERY_POSITION.get()))
            return calculateTargetAngleToPose(FieldConstants.LEFT_DELIVERY_POSITION.get(), currentPosition);
        return calculateTargetAngleToPose(FieldConstants.RIGHT_DELIVERY_POSITION.get(), currentPosition);
    }

    private Rotation2d calculateTargetAngleToPose(Translation2d targetTranslation, Pose2d currentPosition) {
        final Translation2d difference = targetTranslation.minus(currentPosition.getTranslation());
        return difference.getAngle().minus(currentPosition.getRotation());
    }

    private Rotation2d limitAngle(Rotation2d targetAngle) {
        final Rotation2d targetAngleAdjustedToRobotSpeed = getAngleAdjustedForRobotSpeed(targetAngle);

        final Rotation2d[] targetAnglePossibilities = {
                targetAngle,
                Rotation2d.fromDegrees(targetAngle.getDegrees() + 360),
                Rotation2d.fromDegrees(targetAngle.getDegrees() - 360)
        };
        final Rotation2d[] targetAngleAdjustedToRobotSpeedPossibilities = {
                targetAngleAdjustedToRobotSpeed,
                Rotation2d.fromDegrees(targetAngleAdjustedToRobotSpeed.getDegrees() + 360),
                Rotation2d.fromDegrees(targetAngleAdjustedToRobotSpeed.getDegrees() - 360)
        };

        return getBestAngleInRange(targetAnglePossibilities, targetAngleAdjustedToRobotSpeedPossibilities);
    }

    private Rotation2d getBestAngleInRange(Rotation2d[] angleOptions, Rotation2d[] adjustedAngleOptions) {
        final ArrayList<Integer> bothInRangeIndices = new ArrayList<>();
        final ArrayList<Integer> targetAngleInRangeIndices = new ArrayList<>();
        for (int i = 0; i < angleOptions.length; i++) {
            final boolean angleInRange = isAngleInRange(angleOptions[i]);
            final boolean adjustedAngleInRange = isAngleInRange(adjustedAngleOptions[i]);
            if (angleInRange && adjustedAngleInRange)
                bothInRangeIndices.add(i);
            if (angleInRange)
                targetAngleInRangeIndices.add(i);
        }

        if (targetAngleInRangeIndices.isEmpty())
            return getClosestAngleToLimits(adjustedAngleOptions).getDegrees() > TurretConstants.TOTAL_ANGULAR_RANGE.getDegrees() / 2 ? TurretConstants.MAXIMUM_ANGLE : TurretConstants.MINIMUM_ANGLE;
        if (targetAngleInRangeIndices.size() == 1)
            return angleOptions[targetAngleInRangeIndices.get(0)];
        if (bothInRangeIndices.size() == 1)
            return angleOptions[bothInRangeIndices.get(0)];
        return getAngleFurthestFromLimits(angleOptions, bothInRangeIndices);
    }

    private Rotation2d getClosestAngleToLimits(Rotation2d[] angles) {
        Rotation2d bestAngle = angles[0];
        double bestDistanceFromLimit = Math.min(
                Math.abs(bestAngle.getDegrees() - TurretConstants.MINIMUM_ANGLE.getDegrees()),
                Math.abs(bestAngle.getDegrees() - TurretConstants.MAXIMUM_ANGLE.getDegrees())
        );
        for (Rotation2d angle : angles) {
            final double distanceFromMinimumLimit = Math.abs(angle.getDegrees() - TurretConstants.MINIMUM_ANGLE.getDegrees());
            final double distanceFromMaximumLimit = Math.abs(angle.getDegrees() - TurretConstants.MAXIMUM_ANGLE.getDegrees());
            final double distanceFromLimit = Math.min(distanceFromMinimumLimit, distanceFromMaximumLimit);
            if (distanceFromLimit < bestDistanceFromLimit) {
                bestAngle = angle;
                bestDistanceFromLimit = distanceFromLimit;
            }
        }
        return bestAngle;
    }

    private Rotation2d getAngleFurthestFromLimits(Rotation2d[] angles, ArrayList<Integer> indices) {
        Rotation2d bestAngle = angles[indices.get(0)];
        double bestDistanceFromLimit = Math.min(
                Math.abs(bestAngle.getDegrees() - TurretConstants.MINIMUM_ANGLE.getDegrees()),
                Math.abs(bestAngle.getDegrees() - TurretConstants.MAXIMUM_ANGLE.getDegrees())
        );
        for (int i : indices) {
            final double distanceFromMinimumLimit = Math.abs(angles[i].getDegrees() - TurretConstants.MINIMUM_ANGLE.getDegrees());
            final double distanceFromMaximumLimit = Math.abs(angles[i].getDegrees() - TurretConstants.MAXIMUM_ANGLE.getDegrees());
            final double distanceFromLimit = Math.min(distanceFromMinimumLimit, distanceFromMaximumLimit);
            if (distanceFromLimit > bestDistanceFromLimit) {
                bestAngle = angles[i];
                bestDistanceFromLimit = distanceFromLimit;
            }
        }
        return bestAngle;
    }

    private Rotation2d getAngleAdjustedForRobotSpeed(Rotation2d targetAngle) {
        final double currentRobotRotationalSpeedRadiansPerSecond = RobotContainer.SWERVE.getSelfRelativeChassisSpeeds().omegaRadiansPerSecond;
        final Rotation2d velocityAngleChange = Rotation2d.fromRadians(currentRobotRotationalSpeedRadiansPerSecond * TurretConstants.ROBOT_VELOCITY_TO_FUTURE_ANGLE_DEGREES);
        return Rotation2d.fromDegrees(velocityAngleChange.getDegrees() + targetAngle.getDegrees());
    }

    private boolean isAngleInRange(Rotation2d angle) {
        return angle.getDegrees() > TurretConstants.MINIMUM_ANGLE.getDegrees() && angle.getDegrees() < TurretConstants.MAXIMUM_ANGLE.getDegrees();
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, getCurrentEncoderAngle().getRadians())
        );
        return TurretConstants.TURRET_VISUALIZATION_ORIGIN_POINT.transformBy(yawTransform);
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }
}