package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class SimulatedGamePiece {
    private static final ArrayList<SimulatedGamePiece> SIMULATED_GAME_PIECES = new ArrayList<>();
    private static final ArrayList<Rotation2d> CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS = new ArrayList<>();

    private Translation3d fieldRelativePosition;
    private boolean isIndexed = true;
    private Rotation2d spindexerRelativeRotation;
    private Translation3d unindexedRobotRelativeStorePosition;

    public SimulatedGamePiece(double startingPoseXMeters, double startingPoseYMeters) {
        SimulatedGamePieceConstants.GamePieceType gamePieceType = SimulatedGamePieceConstants.GamePieceType.FUEL;
        fieldRelativePosition = new Translation3d(startingPoseXMeters, startingPoseYMeters, gamePieceType.originPointHeightOffGroundMeters);

        SIMULATED_GAME_PIECES.add(this);
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedGamePieces() {
        return SIMULATED_GAME_PIECES;
    }

    public static ArrayList<SimulatedGamePiece> getUnheldGamePieces() {
        final ArrayList<SimulatedGamePiece> unheldGamePieces = new ArrayList<>(SIMULATED_GAME_PIECES);
        unheldGamePieces.removeIf(SimulatedGamePiece::isHeld);

        return unheldGamePieces;
    }

    public void updatePosition(Translation3d fieldRelativePosition) {
        this.fieldRelativePosition = fieldRelativePosition;
    }

    public Translation3d getPosition() {
        return fieldRelativePosition;
    }

    void release() {
        CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS.remove(spindexerRelativeRotation);
        spindexerRelativeRotation = null;
        unindexedRobotRelativeStorePosition = null;
    }

    double getDistanceFromPositionMeters(Translation3d position) {
        return fieldRelativePosition.getDistance(position);
    }

    boolean isIndexed() {
        return isIndexed;
    }

    void resetIndexing() {
        spindexerRelativeRotation = calculateClosestSpindexerRotationFromCurrentPose();
        if (spindexerRelativeRotation == null && unindexedRobotRelativeStorePosition == null) {
            unindexedRobotRelativeStorePosition = calculateRobotRelativeStorePosition();
            isIndexed = false;
            return;
        } else if (spindexerRelativeRotation == null)
            return;

        isIndexed = true;
        CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS.add(spindexerRelativeRotation);
    }

    Rotation2d getSpindexerRelativeRotation() {
        return spindexerRelativeRotation;
    }

    Translation3d getUnindexedRobotRelativeStorePosition() {
        return unindexedRobotRelativeStorePosition;
    }

    static void logAll() {
        Logger.recordOutput("Poses/GamePieces/Fuel", getSimulatedFuelAsPoseArray());
    }

    private static Pose3d[] getSimulatedFuelAsPoseArray() {
        final Pose3d[] poses = new Pose3d[SimulatedGamePiece.SIMULATED_GAME_PIECES.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = new Pose3d(SimulatedGamePiece.SIMULATED_GAME_PIECES.get(i).getPosition(), new Rotation3d());
        return poses;
    }

    private boolean isHeld() {
        return spindexerRelativeRotation != null || unindexedRobotRelativeStorePosition != null;
    }

    private Rotation2d calculateClosestSpindexerRotationFromCurrentPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose3d spindexerRobotRelativePose = RobotContainer.SPINDEXER.calculateComponentPose();
        final Transform3d spindexerRobotRelativeTransform = new Transform3d(
                new Pose3d(),
                spindexerRobotRelativePose
        );

        final double yOffset = new Pose3d(fieldRelativePosition, new Rotation3d()).relativeTo(new Pose3d(robotPose).plus(spindexerRobotRelativeTransform)).getY();
        final double fuelTargetOffsetFromSpindexer = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.toTranslation2d().getNorm();

        final double closestSpindexerRotationRadians = Math.asin(MathUtil.clamp(yOffset / fuelTargetOffsetFromSpindexer, -1, 1));
        final Rotation2d closestSpindexerRotation = Rotation2d.fromRadians(closestSpindexerRotationRadians);
        return findClosestOpenRotationInSpindexer(closestSpindexerRotation, spindexerRobotRelativePose.toPose2d().getRotation());
    }

    //WARNING: 100% vibe coded from this point
    private Translation3d calculateRobotRelativeStorePosition() {
        final Translation3d cornerA = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_START;
        final Translation3d cornerB = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_END;

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

        return new Translation3d(randomX, randomY, randomZ);
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
