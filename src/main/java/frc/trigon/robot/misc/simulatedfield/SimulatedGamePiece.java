package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePiece {
    private static final ArrayList<Rotation2d> CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS = new ArrayList<>(List.of());
    protected SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private Translation3d velocityAtRelease = new Translation3d();
    private double timestampAtRelease = 0;
    private boolean
            isIndexed = true,
            isTouchingGround = true;
    private Rotation2d spindexerRelativeRotation;
    private Pose3d unindexedRobotRelativeStorePose;

    public SimulatedGamePiece(double startingPoseXMeters, double startingPoseYMeters) {
        this.gamePieceType = SimulatedGamePieceConstants.GamePieceType.FUEL;
        fieldRelativePose = new Pose3d(startingPoseXMeters, startingPoseYMeters, this.gamePieceType.originPointHeightOffGroundMeters, new Rotation3d());
    }

    public void updatePeriodically() {
        if (!isIndexed) {
            resetIndexing();
            return;
        }
        if (!isTouchingGround)
            applyGravity();
    }

    /**
     * Releases the game piece from the robot.
     *
     * @param fieldRelativeReleaseVelocity the velocity that the object is released at, relative to the field
     */
    public void release(Translation3d fieldRelativeReleaseVelocity) {
        velocityAtRelease = fieldRelativeReleaseVelocity;
        poseAtRelease = fieldRelativePose;
        timestampAtRelease = Timer.getTimestamp();

        CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS.remove(spindexerRelativeRotation);
        spindexerRelativeRotation = null;
        unindexedRobotRelativeStorePose = null;

        updateIsTouchingGround();
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public double getDistanceFromPoseMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }

    public boolean isIndexed() {
        return isIndexed;
    }

    public void resetIndexing() {
        spindexerRelativeRotation = calculateClosestSpindexerRotationFromCurrentPose();
        if (spindexerRelativeRotation == null && unindexedRobotRelativeStorePose == null) {
            unindexedRobotRelativeStorePose = calculateRobotRelativeStorePose();
            isIndexed = false;
            return;
        } else if (spindexerRelativeRotation == null)
            return;

        isIndexed = true;
        CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS.add(spindexerRelativeRotation);
    }

    public Rotation2d getSpindexerRelativeRotation() {
        return spindexerRelativeRotation;
    }

    public Pose3d getUnindexedRobotRelativeStorePose() {
        return unindexedRobotRelativeStorePose;
    }

    private void applyGravity() {
        if (poseAtRelease == null)
            return;
        final double timeSinceEject = Timer.getTimestamp() - timestampAtRelease;
        this.fieldRelativePose = new Pose3d(poseAtRelease.getTranslation(), new Rotation3d()).transformBy(calculateVelocityPoseTransform(timeSinceEject));

        updateIsTouchingGround();
    }

    private Transform3d calculateVelocityPoseTransform(double elapsedTime) {
        return new Transform3d(
                velocityAtRelease.getX() * elapsedTime,
                velocityAtRelease.getY() * elapsedTime,
                velocityAtRelease.getZ() * elapsedTime - ((SimulatedGamePieceConstants.G_FORCE / 2) * elapsedTime * elapsedTime),
                poseAtRelease.getRotation()
        );
    }

    private void updateIsTouchingGround() {
        if (fieldRelativePose.getZ() < gamePieceType.originPointHeightOffGroundMeters) {
            isTouchingGround = true;
            velocityAtRelease = new Translation3d();

            final Translation3d fieldRelativeTranslation = new Translation3d(
                    fieldRelativePose.getTranslation().getX(),
                    fieldRelativePose.getTranslation().getY(),
                    gamePieceType.originPointHeightOffGroundMeters
            );
            final Rotation3d fieldRelativeRotation = new Rotation3d(
                    fieldRelativePose.getRotation().getX(),
                    0,
                    fieldRelativePose.getRotation().getZ()
            );
            fieldRelativePose = new Pose3d(fieldRelativeTranslation, fieldRelativeRotation);
            return;
        }
        isTouchingGround = false;
    }

    private Rotation2d calculateClosestSpindexerRotationFromCurrentPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose3d spindexerRobotRelativePose = RobotContainer.SPINDEXER.calculateComponentPose();
        final Transform3d spindexerRobotRelativeTransform = new Transform3d(
                new Pose3d(),
                spindexerRobotRelativePose
        );

        final double yOffset = fieldRelativePose.relativeTo(new Pose3d(robotPose).plus(spindexerRobotRelativeTransform)).getY();
        final double fuelTargetOffsetFromSpindexer = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.toTranslation2d().getNorm();

        final double closestSpindexerRotationRadians = Math.asin(MathUtil.clamp(yOffset / fuelTargetOffsetFromSpindexer, -1, 1));
        final Rotation2d closestSpindexerRotation = Rotation2d.fromRadians(closestSpindexerRotationRadians);
        return findClosestOpenRotationInSpindexer(closestSpindexerRotation, spindexerRobotRelativePose.toPose2d().getRotation());
    }

    //WARNING: 100% vibe coded from this point
    private Pose3d calculateRobotRelativeStorePose() {
        final Pose3d cornerA = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_START;
        final Pose3d cornerB = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_END;

        // Calculate radius to ensure the center is far enough from walls
        final double fuelRadius = SimulatedGamePieceConstants.FUEL_DIAMETER_METERS / 2.0;

        // 1. Normalize the bounds (finding the true min and max for X, Y, Z)
        double minX = Math.min(cornerA.getX(), cornerB.getX());
        double maxX = Math.max(cornerA.getX(), cornerB.getX());

        double minY = Math.min(cornerA.getY(), cornerB.getY());
        double maxY = Math.max(cornerA.getY(), cornerB.getY());

        double minZ = Math.min(cornerA.getZ(), cornerB.getZ());
        double maxZ = Math.max(cornerA.getZ(), cornerB.getZ());

        // 2. Inset the boundaries by the fuel radius
        // This prevents the fuel from poking out of the box
        minX += fuelRadius;
        maxX -= fuelRadius;

        minY += fuelRadius;
        maxY -= fuelRadius;

        minZ += fuelRadius;
        maxZ -= fuelRadius;

        // 3. Safety Check: If the box is smaller than the fuel diameter, it can't fit
        if (minX >= maxX || minY >= maxY || minZ >= maxZ) {
            return null;
        }

        // 4. Generate the random position within the safe (shrunk) bounds
        double randomX = minX + (Math.random() * (maxX - minX));
        double randomY = minY + (Math.random() * (maxY - minY));
        double randomZ = minZ + (Math.random() * (maxZ - minZ));

        return new Pose3d(randomX, randomY, randomZ, new Rotation3d());
    }

    private Rotation2d findClosestOpenRotationInSpindexer(Rotation2d targetRotation, Rotation2d currentSpindexerRotation) {
        final ArrayList<Rotation2d> occupiedRotations = CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS;
        final double angularWidthRad = SimulatedGamePieceConstants.FUEL_DIAMETER_METERS /
                SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.toTranslation2d().getNorm();

        final double EPSILON = 0.001;
        final double clearanceRequired = angularWidthRad + EPSILON;

        // 1. Define the Robot-Relative Deadzone (80° to 180°)
        final double ROBOT_FORBIDDEN_MIN = Math.toRadians(80);
        final double ROBOT_FORBIDDEN_MAX = Math.PI; // 180 degrees

        // 2. Convert Robot-Relative bounds to Spindexer-Relative bounds
        // Formula: SpindexerRelative = RobotRelative - SpindexerRotation
        double spindexerForbiddenMin = MathUtil.angleModulus(ROBOT_FORBIDDEN_MIN - currentSpindexerRotation.getRadians());
        double spindexerForbiddenMax = MathUtil.angleModulus(ROBOT_FORBIDDEN_MAX - currentSpindexerRotation.getRadians());

        // 3. Normalize target and check if it's in the dynamic forbidden zone
        double targetRad = MathUtil.angleModulus(targetRotation.getRadians());

        if (isAngleInSector(targetRad, spindexerForbiddenMin, spindexerForbiddenMax)) {
            // Snap to the closer boundary of the deadzone
            double distToMin = Math.abs(MathUtil.angleModulus(targetRad - spindexerForbiddenMin));
            double distToMax = Math.abs(MathUtil.angleModulus(targetRad - spindexerForbiddenMax));
            targetRad = (distToMin < distToMax) ? spindexerForbiddenMin : spindexerForbiddenMax;
        }
        Rotation2d clampedTarget = Rotation2d.fromRadians(targetRad);

        // 4. Check if the clamped target is clear of other fuel
        boolean targetBlocked = false;
        for (Rotation2d occupied : occupiedRotations) {
            if (Math.abs(clampedTarget.minus(occupied).getRadians()) < clearanceRequired) {
                targetBlocked = true;
                break;
            }
        }
        if (!targetBlocked) return clampedTarget;

        // 5. Generate candidate "Exit Points"
        ArrayList<Rotation2d> candidates = new ArrayList<>();
        candidates.add(Rotation2d.fromRadians(spindexerForbiddenMin));
        candidates.add(Rotation2d.fromRadians(spindexerForbiddenMax));

        for (Rotation2d occupied : occupiedRotations) {
            candidates.add(occupied.plus(Rotation2d.fromRadians(clearanceRequired)));
            candidates.add(occupied.minus(Rotation2d.fromRadians(clearanceRequired)));
        }

        Rotation2d bestRotation = null;
        double minDistance = Double.MAX_VALUE;

        for (Rotation2d candidate : candidates) {
            double candRad = MathUtil.angleModulus(candidate.getRadians());

            // FILTER: Discard if the point falls inside the Robot-Relative Deadzone
            if (isAngleInSector(candRad, spindexerForbiddenMin, spindexerForbiddenMax)) {
                continue;
            }

            // Check for collisions with other fuel
            boolean isBlocked = false;
            for (Rotation2d occupied : occupiedRotations) {
                if (Math.abs(candidate.minus(occupied).getRadians()) < clearanceRequired - (EPSILON * 2)) {
                    isBlocked = true;
                    break;
                }
            }

            if (!isBlocked) {
                double dist = Math.abs(clampedTarget.minus(candidate).getRadians());
                if (dist < minDistance) {
                    minDistance = dist;
                    bestRotation = candidate;
                }
            }
        }

        return bestRotation;
    }

    /**
     * Helper to check if an angle is within a sector, handling wrap-around cases.
     */
    private boolean isAngleInSector(double angle, double start, double end) {
        double normalizedStart = MathUtil.angleModulus(start);
        double normalizedEnd = MathUtil.angleModulus(end);
        double normalizedAngle = MathUtil.angleModulus(angle);

        if (normalizedStart < normalizedEnd) {
            return normalizedAngle >= normalizedStart && normalizedAngle <= normalizedEnd;
        } else {
            // Sector crosses the PI/-PI boundary
            return normalizedAngle >= normalizedStart || normalizedAngle <= normalizedEnd;
        }
    }
}
