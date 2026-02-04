package frc.trigon.robot.misc;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

import java.util.Map;
import java.util.function.Supplier;

public class MechanismCameraTransformCalculator {
    private final TimeInterpolatableBuffer<Rotation2d> angleBuffer;
    private final Pose3d mechanismOrigin;
    private final Supplier<Rotation2d> fallbackAngleSupplier;
    private final boolean isYawMechanism;
    private double latestVelocityRotationsPerSecond = 0.0;

    public MechanismCameraTransformCalculator(
            double historyBufferSizeSeconds,
            Pose3d mechanismOrigin,
            Supplier<Rotation2d> fallbackAngleSupplier,
            boolean isYawMechanism) {
        this.angleBuffer = TimeInterpolatableBuffer.createBuffer(Rotation2d::interpolate, historyBufferSizeSeconds);
        this.mechanismOrigin = mechanismOrigin;
        this.fallbackAngleSupplier = fallbackAngleSupplier;
        this.isYawMechanism = isYawMechanism;
    }

    public void update(double[] positions, double[] timestamps, double velocityRotationsPerSecond) {
        if (positions.length != timestamps.length) {
            System.out.println("Mechanism positions and timestamps arrays must have the same length. " +
                    "Positions length: " + positions.length + ", Timestamps length: " + timestamps.length);
            return;
        }

        for (int i = 0; i < positions.length; i++)
            addSample(positions[i], timestamps[i]);
        this.latestVelocityRotationsPerSecond = velocityRotationsPerSecond;
    }

    public Transform3d calculateRobotToCameraAtTime(double timestampSeconds, Transform3d mechanismToCameraTransform) {
        final Rotation2d angle = calculateAngleAtTime(timestampSeconds);
        final Transform3d rotationTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, isYawMechanism ? 0 : -angle.getRadians(), isYawMechanism ? angle.getRadians() : 0)
        );
        final Pose3d rotatedOrigin = mechanismOrigin.plus(rotationTransform);
        final Pose3d cameraPose = rotatedOrigin.plus(mechanismToCameraTransform);
        return cameraPose.minus(new Pose3d());
    }

    private Rotation2d calculateAngleAtTime(double timestampSeconds) {
        final Rotation2d sampledAngle = sampleAngleAtTime(timestampSeconds);
        if (sampledAngle == null)
            return estimateFutureAngle(timestampSeconds);
        return sampledAngle;
    }

    private Rotation2d estimateFutureAngle(double futureTimestampSeconds) {
        final Map.Entry<Double, Rotation2d> latestEntry = getLatestBufferEntry();
        if (latestEntry == null)
            return fallbackAngleSupplier.get();

        final Double latestTimestamp = latestEntry.getKey();
        final Rotation2d latestAngle = latestEntry.getValue();
        final double timeDeltaSeconds = futureTimestampSeconds - latestTimestamp;
        final double predictedRotations = latestVelocityRotationsPerSecond * timeDeltaSeconds;
        return latestAngle.plus(Rotation2d.fromRotations(predictedRotations));
    }

    private Rotation2d sampleAngleAtTime(double timestampSeconds) {
        return angleBuffer.getSample(timestampSeconds).orElse(null);
    }

    private Map.Entry<Double, Rotation2d> getLatestBufferEntry() {
        return angleBuffer.getInternalBuffer().lastEntry();
    }

    private void addSample(double positionRotations, double timestampSeconds) {
        angleBuffer.addSample(timestampSeconds, Rotation2d.fromRotations(positionRotations));
    }
}