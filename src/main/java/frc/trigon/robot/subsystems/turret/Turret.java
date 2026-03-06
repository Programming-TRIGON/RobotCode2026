package frc.trigon.robot.subsystems.turret;

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
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.MechanismCameraTransformCalculator;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Map;

public class Turret extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor
            masterMotor = TurretConstants.MASTER_MOTOR,
            followerMotor = TurretConstants.FOLLOWER_MOTOR;
    private final CANcoderEncoder encoder = TurretConstants.ENCODER;
    private final MechanismCameraTransformCalculator turretCameraTransformCalculator = new MechanismCameraTransformCalculator(
            TurretConstants.TURRET_ANGLE_HISTORY_SIZE_SECONDS,
            TurretConstants.TURRET_ORIGIN_POINT_FOR_CAMERA_CALCULATION,
            this::getCurrentSelfRelativeAngle,
            true
    );
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TurretConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(TurretConstants.FOC_ENABLED).withUpdateFreqHz(1000);
    private double[] latestThreadedPositions = new double[0];
    private Rotation2d targetSelfRelativeAngle = new Rotation2d();
    private int scanForAprilTagsSign = 1;

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
        followerMotor.update();
        encoder.update();

        logTurretStats();
    }

    @Override
    public void updateMechanism() {
        final Rotation2d currentSelfRelativeAngle = getCurrentSelfRelativeAngle();
        final Rotation2d targetProfiledSelfRelativeAngle = Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));

        TurretConstants.MECHANISM.update(
                currentSelfRelativeAngle,
                targetProfiledSelfRelativeAngle
        );

        Logger.recordOutput("Poses/Components/ClosestAprilTagPose", calculateClosestAprilTagPose().get());
        Logger.recordOutput("Poses/Components/TurretPose", calculateVisualizationPose());
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        targetSelfRelativeAngle = new Rotation2d();
    }

    public Transform3d calculateRightCameraTransformAtTime(double timestampSeconds) {
        final Transform3d robotToCameraTransform = turretCameraTransformCalculator.calculateRobotToCameraAtTime(timestampSeconds, TurretConstants.TURRET_TO_RIGHT_CAMERA_TRANSFORM);
        Logger.recordOutput("Turret/RobotToRightCameraTransform", robotToCameraTransform);

        return robotToCameraTransform;
    }

    public Transform3d calculateLeftCameraTransformAtTime(double timestampSeconds) {
        final Transform3d robotToCameraTransform = turretCameraTransformCalculator.calculateRobotToCameraAtTime(timestampSeconds, TurretConstants.TURRET_TO_LEFT_CAMERA_TRANSFORM);
        Logger.recordOutput("Turret/RobotToLeftCameraTransform", robotToCameraTransform);
        return robotToCameraTransform;
    }

    public void updateLatestThreadedPositions() { // TODO: This function and logic are ugly. Find a better way to do this, perhaps in the Phoenix6SignalThread class itself.
        masterMotor.update();
        latestThreadedPositions = masterMotor.getThreadedSignal(TalonFXSignal.POSITION);
    }

    public void updateCameraTransforms() {
        turretCameraTransformCalculator.update(
                latestThreadedPositions,
                Phoenix6SignalThread.getInstance().getLatestTimestamps(),
                masterMotor.getSignal(TalonFXSignal.VELOCITY)
        );
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

    public boolean atTargetShootingCalculationsAngle(boolean useWideTolerance) {
        return atFieldRelativeAngle(shootingCalculations.getTargetShootingState().targetFieldRelativeYaw(), useWideTolerance);
    }

    public boolean atTargetAngle(boolean useWideTolerance) {
        return atSelfRelativeAngle(targetSelfRelativeAngle, useWideTolerance);
    }

    public boolean atFieldRelativeAngle(Rotation2d fieldRelativeAngle, boolean useWideTolerance) {
        final double differenceRadians = Math.abs(fieldRelativeAngle.minus(getCurrentFieldRelativeAngle()).getRadians());
        final double toleranceRadians = useWideTolerance ? TurretConstants.WIDE_TOLERANCE.getRadians() : TurretConstants.NORMAL_TOLERANCE.getRadians();
        return differenceRadians < toleranceRadians;
    }

    public boolean atSelfRelativeAngle(Rotation2d selfRelativeAngle, boolean useWideTolerance) {
        final double differenceRadians = Math.abs(selfRelativeAngle.minus(getCurrentSelfRelativeAngle()).getRadians());
        final double toleranceRadians = useWideTolerance ? TurretConstants.WIDE_TOLERANCE.getRadians() : TurretConstants.NORMAL_TOLERANCE.getRadians();
        return differenceRadians < toleranceRadians;
    }

    public Rotation2d getCurrentSelfRelativeAngle() {
        return Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.POSITION));
    }

    public Translation2d calculateClosestDeliveryPosition() {
        final Pose2d currentPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final double
                distanceFromRightDeliveryPosition = currentPosition.getTranslation().getDistance(FieldConstants.RIGHT_DELIVERY_POSITION.get()),
                distanceFromLeftDeliveryPosition = currentPosition.getTranslation().getDistance(FieldConstants.LEFT_DELIVERY_POSITION.get());
        if (distanceFromRightDeliveryPosition < distanceFromLeftDeliveryPosition)
            return FieldConstants.RIGHT_DELIVERY_POSITION.get();
        return FieldConstants.LEFT_DELIVERY_POSITION.get();
    }

    public void slowScanForAprilTag() {
        final Rotation2d currentAngle = Rotation2d.fromDegrees(getCurrentSelfRelativeAngle().getDegrees() + (10 * scanForAprilTagsSign));

        if (!isAngleInRange(currentAngle))
            scanForAprilTagsSign = -scanForAprilTagsSign;

        masterMotor.setControl(
                voltageRequest.withOutput(TurretConstants.SLOW_SCAN_FOR_APRILTAGS_VOLTAGE * scanForAprilTagsSign).withIgnoreSoftwareLimits(false)
        );
    }

    void alignToHub() {
        final Rotation2d targetFieldRelativeYaw = shootingCalculations.getTargetShootingState().targetFieldRelativeYaw();
        setTargetFieldRelativeAngle(targetFieldRelativeYaw);
    }

    void alignToClosestAprilTag() {
        final Rotation2d targetFieldRelativeAngle = calculateFieldRelativeAngleToClosestAprilTag();

        if (targetFieldRelativeAngle == null)
            return;

        setTargetFieldRelativeAngle(targetFieldRelativeAngle);
    }

    void alignForDelivery() {
        final Rotation2d targetAngle = calculateTargetAngleForDelivery();
        setTargetSelfRelativeAngle(targetAngle);
    }

    void alignForEjection() {
        setTargetSelfRelativeAngle(TurretConstants.SELF_RELATIVE_EJECTION_ANGLE);
    }

    void setTargetFieldRelativeAngle(Rotation2d targetAngle) {
        final Pose2d robotPose = getPredictedRobotPose();
        final Rotation2d targetRobotRelativeAngle = targetAngle.minus(robotPose.getRotation());
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

    private void logTurretStats() {
        Logger.recordOutput("Turret/CurrentSelfRelativeAngleDegrees", getCurrentSelfRelativeAngle().getDegrees());
        Logger.recordOutput("Turret/CurrentFieldRelativeAngleDegrees", getCurrentFieldRelativeAngle().getDegrees());
        Logger.recordOutput("Turret/TargetSelfRelativeAngleDegrees", targetSelfRelativeAngle.getDegrees());
        Logger.recordOutput("Turret/TargetProfiledSelfRelativeAngleDegrees", Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)).getDegrees());
    }

    private Rotation2d calculateFieldRelativeAngleToClosestAprilTag() {
        final Pose2d robotPose = getPredictedRobotPose();
        final FlippablePose2d closestTagToRobotPose = calculateClosestAprilTagPose();

        if (closestTagToRobotPose == null)
            return null;

        return calculateTargetAngleToPose(
                closestTagToRobotPose.get().getTranslation(),
                robotPose
        ).plus(robotPose.getRotation());
    }

    private FlippablePose2d calculateClosestAprilTagPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();

        double closestTagDistanceToRobotMeters = Double.POSITIVE_INFINITY;
        FlippablePose2d closestTagPose = null;
        int closestTagID = 0;

        for (Map.Entry<Integer, Pose3d> tagIdToPose : FieldConstants.TAG_ID_TO_POSE.entrySet()) {
            final int tagID = tagIdToPose.getKey();
            final Pose2d tagPose = tagIdToPose.getValue().toPose2d();

            final FlippablePose2d flippableTagPose = new FlippablePose2d(tagPose, false);
            final Pose2d fieldRelativeTagPose = flippableTagPose.get();
            final double currentTagDistanceToRobotMeters = robotPose.getTranslation().getDistance(fieldRelativeTagPose.getTranslation());

            if (currentTagDistanceToRobotMeters < closestTagDistanceToRobotMeters) {
                closestTagDistanceToRobotMeters = currentTagDistanceToRobotMeters;
                closestTagPose = flippableTagPose;
                closestTagID = tagID;
            }
        }

        Logger.recordOutput("Turret/ClosestAprilTagID", closestTagID);
        return closestTagPose;
    }


    private double calculateResistSwerveRotationFeedforward() {
        final double robotRotationalVelocityRadiansPerSecond = RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond();
        final double robotRotationalVelocityRotationsPerSecond = robotRotationalVelocityRadiansPerSecond / (2 * Math.PI);
        return -robotRotationalVelocityRotationsPerSecond * TurretConstants.RESIST_SWERVE_ROTATION_FEEDFORWARD_GAIN;
    }

    private Rotation2d calculateTargetAngleForDelivery() {
        final Pose2d currentPosition = getPredictedRobotPose();
        final Rotation2d angleToDeliveryPoint = calculateTargetAngleToPose(calculateClosestDeliveryPosition(), currentPosition);
        final double currentYVelocity = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds().vyMetersPerSecond;
        final double currentAllianceYVelocity = Flippable.isRedAlliance() ? -currentYVelocity : currentYVelocity;
        final Rotation2d yVelocityResistanceAngle = Rotation2d.fromDegrees(currentAllianceYVelocity * TurretConstants.RESIST_Y_MOVEMENT_FOR_DELIVERY_COEFFICIENT);
        return angleToDeliveryPoint.plus(yVelocityResistanceAngle);
    }

    private Pose2d getPredictedRobotPose() {
        final Pose2d currentPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose2d predictedPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TurretConstants.ROBOT_ROTATION_PREDICTION_TIME_SECONDS);
        Logger.recordOutput("Turret/PredictedDiffDegrees", currentPose.getRotation().minus(predictedPose.getRotation()).getDegrees());
        return predictedPose;
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