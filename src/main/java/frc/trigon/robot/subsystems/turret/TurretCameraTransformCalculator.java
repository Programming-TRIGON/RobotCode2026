package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.trigon.robot.RobotContainer;

import java.util.Map;

public class TurretCameraTransformCalculator {
    private static TurretCameraTransformCalculator INSTANCE = null;
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer.createBuffer(Rotation2d::interpolate, TurretConstants.TURRET_ANGLE_HISTORY_SIZE_SECONDS);
    private double latestVelocityRotationsPerSecond = 0.0;

    public static TurretCameraTransformCalculator getInstance() {
        if (INSTANCE == null)
            INSTANCE = new TurretCameraTransformCalculator();
        return INSTANCE;
    }

    private TurretCameraTransformCalculator() {
    }

    public void update(double[] turretPositions, double[] timestamps, double velocityRotationsPerSecond) {
        if (turretPositions.length > timestamps.length) {
            System.out.println("Turret positions and timestamps arrays must have the same length.");
            return;
        }

        for (int i = 0; i < turretPositions.length; i++)
            addSample(turretPositions[i], timestamps[i]);
        this.latestVelocityRotationsPerSecond = velocityRotationsPerSecond;
    }

    public Transform3d calculateRobotToRightCameraAtTime(double timestampSeconds) {
        return calculateRobotToCameraAtTime(timestampSeconds, TurretConstants.TURRET_TO_RIGHT_CAMERA_TRANSFORM);
    }

    public Transform3d calculateRobotToLeftCameraAtTime(double timestampSeconds) {
        return calculateRobotToCameraAtTime(timestampSeconds, TurretConstants.TURRET_TO_LEFT_CAMERA_TRANSFORM);
    }

    private Transform3d calculateRobotToCameraAtTime(double timestampSeconds, Transform3d turretToCameraTransform) {
        final Rotation2d turretAngle = calculateTurretAngleAtTime(timestampSeconds);
        if (turretAngle == null)
            return null;

        final Transform3d turretAngleTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, turretAngle.getRadians())
        );
        final Pose3d rotatedTurretOrigin = TurretConstants.TURRET_ORIGIN_POINT_FOR_CAMERA_CALCULATION.plus(turretAngleTransform);
        final Pose3d cameraPose = rotatedTurretOrigin.plus(turretToCameraTransform);
        return cameraPose.minus(new Pose3d());
    }

    private Rotation2d calculateTurretAngleAtTime(double timestampSeconds) {
        if (isTimestampTooNew(timestampSeconds))
            return estimateFutureTurretAngle(timestampSeconds);

        return sampleTurretAngleAtTime(timestampSeconds);
    }

    private boolean isTimestampTooNew(double timestampSeconds) {
        final Map.Entry<Double, Rotation2d> latestBufferEntry = getLatestBufferEntry();
        if (latestBufferEntry == null)
            return false;
        final Double latestTimestamp = latestBufferEntry.getKey();
        return timestampSeconds > latestTimestamp;
    }

    private Rotation2d estimateFutureTurretAngle(double futureTimestampSeconds) {
        final Map.Entry<Double, Rotation2d> latestBufferEntry = getLatestBufferEntry();
        if (latestBufferEntry == null)
            return RobotContainer.TURRET.getCurrentSelfRelativeAngle();
        final Double latestTimestamp = latestBufferEntry.getKey();
        final Rotation2d latestAngle = latestBufferEntry.getValue();

        final double timeDeltaSeconds = futureTimestampSeconds - latestTimestamp;
        final double predictedRotations = latestVelocityRotationsPerSecond * timeDeltaSeconds;
        return latestAngle.plus(Rotation2d.fromRotations(predictedRotations));
    }

    private Rotation2d sampleTurretAngleAtTime(double timestampSeconds) {
        return turretAngleBuffer.getSample(timestampSeconds).orElse(null);
    }

    private Map.Entry<Double, Rotation2d> getLatestBufferEntry() {
        return turretAngleBuffer.getInternalBuffer().lastEntry();
    }

    private void addSample(double turretPositionRotations, double timestampSeconds) {
        turretAngleBuffer.addSample(timestampSeconds, Rotation2d.fromRotations(turretPositionRotations));
    }
}
