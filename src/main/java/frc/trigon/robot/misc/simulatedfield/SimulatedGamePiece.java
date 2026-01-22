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
        if (spindexerRelativeRotation == null) {
            unindexedRobotRelativeStorePose = calculateRobotRelativeStorePose();
            isIndexed = false;
            return;
        }
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

        return findClosestOpenRotationInSpindexer(closestSpindexerRotation);
    }

    private Pose3d calculateRobotRelativeStorePose() {
        return new Pose3d();
    }

    //WARNING: 100% vibe coded from this point
    private Rotation2d findClosestOpenRotationInSpindexer(Rotation2d targetRotation) {
        final ArrayList<Rotation2d> occupiedRotations = CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS;
        final double angularWidthRad = SimulatedGamePieceConstants.FUEL_DIAMETER_METERS /
                SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.toTranslation2d().getNorm();

        // We use a small epsilon to ensure balls aren't "frame-perfect" touching
        final double EPSILON = 0.001;
        final double clearanceRequired = angularWidthRad + EPSILON;

        if (occupiedRotations.isEmpty()) return targetRotation;

        // 1. Check if the target is already valid
        boolean targetBlocked = false;
        for (Rotation2d occupied : occupiedRotations) {
            if (Math.abs(targetRotation.minus(occupied).getRadians()) < clearanceRequired) {
                targetBlocked = true;
                break;
            }
        }
        if (!targetBlocked) return targetRotation;

        // 2. Collect all potential "Exit Points" (Edges of every ball)
        ArrayList<Rotation2d> potentialExitPoints = new ArrayList<>();
        for (Rotation2d occupied : occupiedRotations) {
            potentialExitPoints.add(occupied.plus(Rotation2d.fromRadians(clearanceRequired)));
            potentialExitPoints.add(occupied.minus(Rotation2d.fromRadians(clearanceRequired)));
        }

        // 3. Filter the exit points. An exit point is only valid if it doesn't
        // collide with ANY of the existing balls.
        ArrayList<Rotation2d> validExitPoints = new ArrayList<>();
        for (Rotation2d exitPoint : potentialExitPoints) {
            boolean isPointBlocked = false;
            for (Rotation2d occupied : occupiedRotations) {
                // We use a slightly smaller check here (minus epsilon) to allow
                // the point to exist exactly on the edge of the ball that created it
                if (Math.abs(exitPoint.minus(occupied).getRadians()) < clearanceRequired - (EPSILON * 2)) {
                    isPointBlocked = true;
                    break;
                }
            }
            if (!isPointBlocked) {
                validExitPoints.add(exitPoint);
            }
        }

        // 4. From the valid exit points, find the one closest to the targetRotation
        Rotation2d bestRotation = null;
        double minDistance = Double.MAX_VALUE;

        for (Rotation2d validPoint : validExitPoints) {
            double dist = Math.abs(targetRotation.minus(validPoint).getRadians());
            if (dist < minDistance) {
                minDistance = dist;
                bestRotation = validPoint;
            }
        }

        // 5. If no valid points are found, the spindexer is likely full.
        // Otherwise, return the closest point that gets us out of the "stack."
        return bestRotation;
    }
}
