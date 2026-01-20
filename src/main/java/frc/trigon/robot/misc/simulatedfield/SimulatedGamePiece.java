package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class SimulatedGamePiece {
    private static ArrayList<Rotation2d> CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS = new ArrayList<>(List.of());
    protected SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private Translation3d velocityAtRelease = new Translation3d();
    private double timestampAtRelease = 0;
    private boolean
            isIndexed = true,
            isTouchingGround = true;
    private Rotation2d spindexerRelativeRotation;

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
            isIndexed = false;
            return;
        }
        isIndexed = true;
        CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS.add(spindexerRelativeRotation);
    }

    public Rotation2d getSpindexerRelativeRotation() {
        return spindexerRelativeRotation;
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
        final Pose3d spindexerPose = RobotContainer.SPINDEXER.calculateComponentPose();
        final double yOffset = spindexerPose.getY() - fieldRelativePose.getY();
        final double fuelTargetOffsetFromSpindexer = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.getNorm();

        final double closestSpindexerRotationRadians = Math.asin(yOffset / fuelTargetOffsetFromSpindexer);
        if (Double.isNaN(closestSpindexerRotationRadians))
            return null;
        final Rotation2d closestSpindexerRotation = Rotation2d.fromRadians(closestSpindexerRotationRadians);

        return findClosestOpenRotationInSpindexer(closestSpindexerRotation);
    }

    private Rotation2d findClosestOpenRotationInSpindexer(Rotation2d targetRotation) {
        final ArrayList<Rotation2d> occupiedRotations = CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS;
        final double
                fuelDiameterMeters = SimulatedGamePieceConstants.FUEL_DIAMETER_METERS,
                spindexerRadiusMeters = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS.toTranslation2d().getNorm();
        final double halfFuelSpacingRadius = fuelDiameterMeters / spindexerRadiusMeters / 2;

        if (occupiedRotations.isEmpty()) return targetRotation;

        // 1. Define Forbidden Ranges [-PI, PI]
        List<double[]> forbiddenRanges = new ArrayList<>();
        for (Rotation2d pos : occupiedRotations) {
            double center = pos.getRadians();
            forbiddenRanges.add(new double[]{
                    MathUtil.angleModulus(center - halfFuelSpacingRadius),
                    MathUtil.angleModulus(center + halfFuelSpacingRadius)
            });
        }

        // 2. Identify "Gaps" (Legal Zones)
        // Sort by start angle
        forbiddenRanges.sort(Comparator.comparingDouble(a -> a[0]));

        List<double[]> legalGaps = new ArrayList<>();
        // Simple check: if the entire circle is covered, return null
        if (halfFuelSpacingRadius * 2 * occupiedRotations.size() >= 2 * Math.PI) {
            return null;
        }

        // Find gaps between sorted forbidden ranges
        for (int i = 0; i < forbiddenRanges.size(); i++) {
            double endOfCurrent = forbiddenRanges.get(i)[1];
            double startOfNext = forbiddenRanges.get((i + 1) % forbiddenRanges.size())[0];

            // Handle wrap around PI to -PI
            double gapSize = MathUtil.angleModulus(startOfNext - endOfCurrent);
            if (gapSize > 0) {
                legalGaps.add(new double[]{endOfCurrent, startOfNext});
            }
        }

        // 3. Find closest point in any legal gap to target
        double target = targetRotation.getRadians();
        double bestPoint = Double.NaN;
        double minDiff = Double.MAX_VALUE;

        for (double[] gap : legalGaps) {
            // Check if target is inside the gap
            if (isAngleInGap(target, gap[0], gap[1])) {
                return targetRotation; // Target is already legal!
            }

            // Otherwise, the closest points are the boundaries of the gap
            double[] boundaries = {gap[0], gap[1]};
            for (double b : boundaries) {
                double diff = Math.abs(MathUtil.angleModulus(b - target));
                if (diff < minDiff) {
                    minDiff = diff;
                    bestPoint = b;
                }
            }
        }

        return Double.isNaN(bestPoint) ? null : Rotation2d.fromRadians(bestPoint);
    }

    private static boolean isAngleInGap(double angle, double start, double end) {
        double totalGap = MathUtil.angleModulus(end - start);
        double angleToStart = MathUtil.angleModulus(angle - start);
        return angleToStart >= 0 && angleToStart <= totalGap;
    }
}
