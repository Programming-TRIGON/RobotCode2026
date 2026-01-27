package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class Turret extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor
            masterMotor = TurretConstants.MASTER_MOTOR,
            followerMotor = TurretConstants.FOLLOWER_MOTOR;
    private final CANcoderEncoder encoder = TurretConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TurretConstants.FOC_ENABLED);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(TurretConstants.FOC_ENABLED).withUpdateFreqHz(1000);
    private Rotation2d targetSelfRelativeAngle = Rotation2d.fromDegrees(0);

    public Turret() {
        setName("Turret");
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        masterMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("TurretMotor")
                .angularPosition(Units.Rotations.of(masterMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return TurretConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        followerMotor.update();
        encoder.update();
    }

    @Override
    public void updateMechanism() {
        final Rotation2d currentSelfRelativeAngle = getCurrentSelfRelativeAngle();
        final Rotation2d targetProfiledSelfRelativeAngle = Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
        TurretConstants.MECHANISM.update(
                currentSelfRelativeAngle,
                targetProfiledSelfRelativeAngle
        );
        Logger.recordOutput("Poses/Components/TurretPose", calculateVisualizationPose());

        Logger.recordOutput("Turret/CurrentSelfRelativeAngleDegrees", currentSelfRelativeAngle.getDegrees());
        Logger.recordOutput("Turret/CurrentFieldRelativeAngleDegrees", getCurrentFieldRelativeAngle().getDegrees());
        Logger.recordOutput("Turret/TargetSelfRelativeAngleDegrees", targetSelfRelativeAngle.getDegrees());
        Logger.recordOutput("Turret/TargetProfiledSelfRelativeAngle", targetProfiledSelfRelativeAngle.getDegrees());
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    public Pose3d calculateVisualizationPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, getCurrentSelfRelativeAngle().getRadians())
        );
        return TurretConstants.TURRET_VISUALIZATION_ORIGIN_POINT.transformBy(yawTransform);
    }

    public Rotation2d getTargetFieldRelativeAngle() {
        return targetSelfRelativeAngle.plus(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation());
    }

    public Rotation2d getTargetSelfRelativeAngle() {
        return targetSelfRelativeAngle;
    }

    public Rotation2d getCurrentFieldRelativeAngle() {
        return getCurrentSelfRelativeAngle().plus(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation());
    }

    public Rotation2d getCurrentSelfRelativeAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    void alignToHub() {
        final Rotation2d targetFieldRelativeYaw = shootingCalculations.getTargetShootingState().targetFieldRelativeYaw();
        setTargetFieldRelativeAngle(targetFieldRelativeYaw);
    }

    void alignToClosestAprilTag() {
        // TODO: add logic
    }

    void alignForDelivery() {
        final Rotation2d targetAngle = calculateTargetAngleForDelivery();
        setTargetSelfRelativeAngle(targetAngle);
    }

    void setTargetFieldRelativeAngle(Rotation2d targetAngle) {
        final Rotation2d targetRobotRelativeAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getDegrees());
        setTargetSelfRelativeAngle(targetRobotRelativeAngle);
    }

    void setTargetSelfRelativeAngle(Rotation2d targetAngle) {
        targetSelfRelativeAngle = limitAngle(targetAngle);
        final double resistSwerveRotationFeedforward = calculateResistSwerveRotationFeedforward();
        masterMotor.setControl(positionRequest
                .withPosition(targetSelfRelativeAngle.getRotations())
                .withFeedForward(resistSwerveRotationFeedforward)
        );
    }

    private double calculateResistSwerveRotationFeedforward() {
        final double robotRotationalVelocityRadiansPerSecond = RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond();
        final double robotRotationalVelocityRotationsPerSecond = robotRotationalVelocityRadiansPerSecond / (2 * Math.PI);
        return -robotRotationalVelocityRotationsPerSecond * TurretConstants.RESIST_SWERVE_ROTATION_FEEDFORWARD_GAIN;
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
            final double distanceFromLimit = getDistanceFromLimits(angle);
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
            final double distanceFromLimit = getDistanceFromLimits(angles[i]);
            if (distanceFromLimit > bestDistanceFromLimit) {
                bestAngle = angles[i];
                bestDistanceFromLimit = distanceFromLimit;
            }
        }
        return bestAngle;
    }

    private double getDistanceFromLimits(Rotation2d angle) {
        return Math.min(
                Math.abs(angle.getDegrees() - TurretConstants.MINIMUM_ANGLE.getDegrees()),
                Math.abs(angle.getDegrees() - TurretConstants.MAXIMUM_ANGLE.getDegrees())
        );
    }

    private Rotation2d getAngleAdjustedForRobotSpeed(Rotation2d targetAngle) {
        final double currentRobotRotationalSpeedRadiansPerSecond = RobotContainer.SWERVE.getSelfRelativeChassisSpeeds().omegaRadiansPerSecond;
        final Rotation2d velocityAngleChange = Rotation2d.fromRadians(currentRobotRotationalSpeedRadiansPerSecond * TurretConstants.ROBOT_VELOCITY_TO_FUTURE_ANGLE_SECONDS);
        return Rotation2d.fromDegrees(velocityAngleChange.getDegrees() + targetAngle.getDegrees());
    }

    private boolean isAngleInRange(Rotation2d angle) {
        return angle.getDegrees() > TurretConstants.MINIMUM_ANGLE.getDegrees() && angle.getDegrees() < TurretConstants.MAXIMUM_ANGLE.getDegrees();
    }
}