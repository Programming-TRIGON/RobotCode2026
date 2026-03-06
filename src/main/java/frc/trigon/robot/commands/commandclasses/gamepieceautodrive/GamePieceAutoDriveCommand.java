package frc.trigon.robot.commands.commandclasses.gamepieceautodrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.autonomous.SafeAutonomousDriveCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.objectdetection.ObjectPoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class GamePieceAutoDriveCommand extends ParallelCommandGroup {
    private static final ObjectPoseEstimator OBJECT_POSE_ESTIMATOR = RobotContainer.OBJECT_POSE_ESTIMATOR;
    private final AtomicReference<GamePieceCluster> currentTargetCluster = new AtomicReference<>();
    // Raw approach heading (optionally rate-limited).  NOT wall-clamped — the
    // clamp is applied only in getTargetHeading() so the position clamp can
    // still see the unclamped heading.
    private final AtomicReference<FlippableRotation2d> commandedHeading = new AtomicReference<>();
    // Keeps the last heading that was actually valid so that brief vision gaps
    // (cluster == null for a cycle) don't send null to the swerve and release
    // heading control, letting the robot spin freely.
    private volatile FlippableRotation2d lastValidHeading = null;
    /**
     * When true, drive speed ramps down and rotation is rate-limited near the piece.
     */
    private final boolean slowIntake;

    public GamePieceAutoDriveCommand(boolean slowIntake) {
        this.slowIntake = slowIntake;
        addCommands(
                createTargetUpdateCommand(),
                createDriveCommand()
        );
    }

    private Command createTargetUpdateCommand() {
        return new RunCommand(() -> {
            GamePieceCluster prev = currentTargetCluster.get();
            GamePieceCluster bestCluster = findBestCluster();
            currentTargetCluster.set(bestCluster);
            if (prev == null && bestCluster != null) {
                AutonomousConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.reset();
                AutonomousConstants.GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER.reset();
            }
            updateCommandedHeading();
            Logger.recordOutput("GamePieceAutoDrive/hasCollectableGamePiece", hasCollectableGamePiecesInView());
        });
    }

    private Command createDriveCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                this::getXControllerOutput,
                this::getYControllerOutput,
                this::getTargetHeading
        );
    }

    // =========================================================================
    // HEADING
    // =========================================================================

    private void updateCommandedHeading() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null)
            // Don't null out — keep last heading so the swerve maintains
            // orientation during brief vision gaps instead of spinning freely.
            return;

        FlippableRotation2d target = new FlippableRotation2d(cluster.getApproachHeading(), false);

        if (!slowIntake || !isWithinIntakeDistance()) {
            commandedHeading.set(target);
            lastValidHeading = target;
            return;
        }

        // Rate-limit inside the intake zone.
        FlippableRotation2d prev = commandedHeading.get();
        if (prev == null) {
            commandedHeading.set(target);
            lastValidHeading = target;
            return;
        }

        double maxDeltaDeg = GamePieceAutoDriveConstants.MAX_INTAKE_ROTATION_RATE_DEG_PER_SEC * 0.02;
        double deltaDeg = Math.IEEEremainder(target.get().getDegrees() - prev.get().getDegrees(), 360.0);
        if (Math.abs(deltaDeg) > maxDeltaDeg)
            deltaDeg = Math.signum(deltaDeg) * maxDeltaDeg;

        FlippableRotation2d result = new FlippableRotation2d(Rotation2d.fromDegrees(prev.get().getDegrees() + deltaDeg), false);
        commandedHeading.set(result);
        lastValidHeading = result;
    }

    /**
     * What actually goes to the swerve.  The wall clamp is applied HERE so
     * that commandedHeading stays clean for the position clamp to read.
     */
    private FlippableRotation2d getTargetHeading() {
        FlippableRotation2d heading = commandedHeading.get();
        if (heading == null)
            heading = lastValidHeading;
        Logger.recordOutput("GamePieceAutoDrive/RawApproachHeadingDeg", heading == null ? 99999999 : heading.get().getDegrees());
        return clampHeadingForWall(heading);
    }

    /**
     * Prevents the intake from hitting the wall by blocking headings that
     * would swing the intake tip past the effective wall.
     * <p>
     * ── Geometry (intake at BACK, i.e. 180° in robot frame) ──
     * The intake tip is (ROBOT_HALF_WIDTH + INTAKE_REACH) behind the robot
     * center.  In world frame:
     * tipX = robotX  –  intakeLength * cos(heading)
     * <p>
     * Require tipX >= effectiveWallX:
     * cos(heading)  <=  (robotX – effectiveWallX) / intakeLength  =  cosMax
     * <p>
     * cos(θ) <= cosMax is satisfied when |θ| >= acos(cosMax).
     * So the FORBIDDEN zone is  |heading| < acos(cosMax)  (near 0°, where the
     * intake points toward the wall).
     * The ALLOWED zone is everything else (near ±180°, intake away from wall).
     * <p>
     * If the heading is in the forbidden zone it is pushed to the nearest
     * boundary (± acos(cosMax)), which keeps it as close to the desired
     * approach as possible while staying safe.
     */
    private FlippableRotation2d clampHeadingForWall(FlippableRotation2d heading) {
        if (heading == null)
            return null;

        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        double intakeLength = GamePieceAutoDriveConstants.ROBOT_HALF_WIDTH + GamePieceAutoDriveConstants.INTAKE_REACH;

        double cosMax = (robotPose.getX() - getEffectiveWallX()) / intakeLength;

        // Robot is far enough that even heading = 0° (intake straight at wall)
        // keeps the tip clear.  No clamping needed.
        if (cosMax >= 1.0)
            return heading;

        // Robot is so close that NO heading is safe.  Face 180° (intake
        // maximally away from wall) as best-effort emergency.
        if (cosMax < -1.0)
            return new FlippableRotation2d(Rotation2d.fromDegrees(180), false);

        // Normal case: forbidden zone is |heading| < minAbsAngle.
        double minAbsAngleDeg = Math.toDegrees(Math.acos(cosMax));
        double headingDeg = Math.IEEEremainder(heading.get().getDegrees(), 360.0);

        if (Math.abs(headingDeg) < minAbsAngleDeg)
            // In the forbidden zone — push to the nearest allowed boundary.
            // signum preserves which side of 0° we were on so the robot doesn't
            // snap across; exact 0° defaults to +minAbsAngle.
            headingDeg = (headingDeg >= 0) ? minAbsAngleDeg : -minAbsAngleDeg;

        return new FlippableRotation2d(Rotation2d.fromDegrees(headingDeg), false);
    }

    // =========================================================================
    // DRIVE OUTPUTS
    // =========================================================================

    private double getXControllerOutput() {
        if (!shouldDrive())
            return 0.0;
        Translation2d error = getTranslationError();
        if (error == null)
            return 0.0;
        double output = AutonomousConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.calculate(-error.getX());
        Logger.recordOutput("GamePieceAutoDrive/X", output * getIntakeSpeedScale());
        return output * getIntakeSpeedScale();
    }

    private double getYControllerOutput() {
        if (!shouldDrive())
            return 0.0;
        Translation2d error = getTranslationError();
        if (error == null)
            return 0.0;
        double output = AutonomousConstants.GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER.calculate(-error.getY());
        Logger.recordOutput("GamePieceAutoDrive/Y", output * getIntakeSpeedScale());
        return output * getIntakeSpeedScale();
    }

    /**
     * Drive is active as long as there is a reachable target and the robot
     * hasn't closed to intake distance.  No emergency "past-the-wall stop"
     * here: if the robot has overshot the clamped target, the target is now
     * behind it and the PID error naturally drives it back.  Stopping would
     * lock it in the unsafe position.
     */
    private boolean shouldDrive() {
        // FIX: Check if we actually have a valid cluster in the reference
        if (currentTargetCluster.get() == null)
            return false;

        Translation2d error = getTranslationError();
        if (error == null)
            return false;

        return !(error.getNorm() <= AutonomousConstants.AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS);
    }

    /**
     * Linear ramp from 1.0 (at INTAKE_SLOWDOWN_DISTANCE) down to
     * INTAKE_DRIVE_SPEED_SCALE (at distance 0).  Only active when slowIntake
     * is enabled.
     */
    private double getIntakeSpeedScale() {
        if (!slowIntake)
            return 1.0;

        Translation2d error = getTranslationError();
        if (error == null)
            return 1.0;

        double dist = error.getNorm();
        double slowdownDist = GamePieceAutoDriveConstants.INTAKE_SLOWDOWN_DISTANCE_METERS;
        if (dist >= slowdownDist)
            return 1.0;

        double t = dist / slowdownDist;
        return GamePieceAutoDriveConstants.INTAKE_DRIVE_SPEED_SCALE + t * (1.0 - GamePieceAutoDriveConstants.INTAKE_DRIVE_SPEED_SCALE);
    }

    private boolean isWithinIntakeDistance() {
        Translation2d error = getTranslationError();
        return error != null && error.getNorm() < GamePieceAutoDriveConstants.INTAKE_SLOWDOWN_DISTANCE_METERS;
    }

    // =========================================================================
    // WALL SAFETY & TARGET CLAMPING
    // =========================================================================

    /**
     * The real wall shifted inward by the safety margin.  All geometry
     * checks use this as "the wall" so the robot + intake never get closer
     * than WALL_SAFETY_MARGIN_METERS to the actual wall.
     */
    private static double getEffectiveWallX() {
        return GamePieceAutoDriveConstants.ALLIANCE_WALL_X_METERS + 0.05;
    }

    /**
     * Cluster centroid clamped in X so the robot is safe at ANY heading.
     * <p>
     * Uses the worst-case heading (0° = intake pointing directly at the wall,
     * because the intake is at the back).  This is the global maximum of minX
     * across all headings, so it guarantees safety regardless of what heading
     * the robot passes through while rotating.  Only pieces near the wall are
     * affected; pieces further out are unclamped.
     * <p>
     * If the robot has overshot (is closer than the clamped target), the
     * target is behind it and the PID drives it back automatically.
     */
    private Translation2d getEffectiveTarget() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null)
            return null;

        Translation2d target = cluster.getCentroid();
        // Worst case heading for intake-at-back is 0° (intake straight at wall).
        double minX = getMinSafeRobotX(Rotation2d.fromDegrees(0));

        if (target.getX() < minX)
            target = new Translation2d(minX, target.getY());
        return target;
    }

    /**
     * Minimum robot-centre X that keeps the body and intake clear of the
     * effective wall, for a given heading.
     * <p>
     * ── Body ──
     * Square, half-width ROBOT_HALF_WIDTH.  The farthest any corner extends
     * in –X from centre = halfWidth × (|cos θ| + |sin θ|).
     * <p>
     * ── Intake (at BACK = 180° in robot frame) ──
     * Tip is intakeLength behind centre.  In world frame:
     * tipX = robotX – intakeLength × cos(θ)
     * The tip extends toward –X (the wall) when cos(θ) > 0, i.e. |θ| < 90°.
     * The backward (toward-wall) extent is therefore max(0, intakeLength × cos(θ)).
     */
    private double getMinSafeRobotX(Rotation2d heading) {
        double cosH = Math.abs(heading.getCos());
        double sinH = Math.abs(heading.getSin());

        double bodyBackwardExtent = GamePieceAutoDriveConstants.ROBOT_HALF_WIDTH * (cosH + sinH);

        // Intake at back: extends toward the wall when cos(heading) > 0.
        double intakeBackwardExtent = Math.max(0.0,
                (GamePieceAutoDriveConstants.ROBOT_HALF_WIDTH + GamePieceAutoDriveConstants.INTAKE_REACH)
                        * heading.getCos());

        return getEffectiveWallX() + Math.max(bodyBackwardExtent, intakeBackwardExtent);
    }

    /**
     * Robot-frame translation error to the effective (wall-clamped) target.
     */
    private Translation2d getTranslationError() {
        Translation2d effectiveTarget = getEffectiveTarget();
        if (effectiveTarget == null)
            return null;

        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        Translation2d fieldRelativeError = effectiveTarget.minus(robotPose.getTranslation());
        return fieldRelativeError.rotateBy(robotPose.getRotation().unaryMinus());
    }

    // =========================================================================
    // VISION LOGIC
    // =========================================================================

    /**
     * Checks whether there are any collectable game pieces currently visible.
     * A piece is considered collectable if it's within the allowed collection zone
     * (not past mid-field for blue alliance, or not behind the alliance wall for red)
     * AND if it adheres to the Alliance Zone/Neutral Zone logic relative to the robot.
     *
     * @return true if at least one collectable game piece is in view
     */
    public static boolean hasCollectableGamePiecesInView() {
        List<Translation2d> allObjects = OBJECT_POSE_ESTIMATOR.getObjectsOnField();
        if (allObjects.isEmpty())
            return false;

        for (Translation2d piece : allObjects) {
            if (!isOutOfBounds(piece))
                return true;
        }
        return false;
    }

    /**
     * Determines if a game piece is out of bounds based on global field limits
     * AND the robot's current zone (Alliance vs Neutral).
     */
    private static boolean isOutOfBounds(Translation2d piece) {
        // 1. Check Global Field Limits
        final double minX;
        final double maxX;

        if (Flippable.isRedAlliance()) {
            // Red Alliance: Objects are valid from the far right wall (FieldLength) down to the center line.
            minX = FieldConstants.FIELD_LENGTH_METERS - GamePieceAutoDriveConstants.MAX_COLLECTION_X_METERS;
            maxX = FieldConstants.FIELD_LENGTH_METERS;
        } else {
            // Blue Alliance: Objects are valid from the far left wall (0) up to the center line.
            minX = GamePieceAutoDriveConstants.ALLIANCE_WALL_X_METERS;
            maxX = GamePieceAutoDriveConstants.MAX_COLLECTION_X_METERS;
        }

        if (piece.getX() < minX || piece.getX() > maxX)
            return true;

        // 2. Zone-Specific Logic (Alliance vs Neutral)
        // The robot should only collect pieces located in the same zone type (Alliance or Neutral) as itself.
        final boolean isRobotInAllianceZone = FieldConstants.isInAllianceZone();
        final boolean isPieceInAllianceZone = FieldConstants.isPoseInAllianceZone(piece);

        return isRobotInAllianceZone != isPieceInAllianceZone;
    }

    private GamePieceCluster findBestCluster() {
        List<Translation2d> allObjects = OBJECT_POSE_ESTIMATOR.getObjectsOnField();
        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        if (allObjects.isEmpty())
            return null;

        GamePieceCluster bestCluster = null;
        double maxScore = -Double.MAX_VALUE;

        for (Translation2d seed : allObjects) {
            if (isOutOfBounds(seed))
                continue;

            List<Translation2d> clusterPieces = getNeighbors(seed, allObjects);
            if (clusterPieces.isEmpty())
                continue;

            GamePieceCluster cluster = new GamePieceCluster(clusterPieces, robotPose);
            double score = calculateClusterScore(clusterPieces.size(), cluster.getDistanceToRobot());

            if (score > maxScore) {
                maxScore = score;
                bestCluster = cluster;
            }
        }
        return bestCluster;
    }

    private List<Translation2d> getNeighbors(Translation2d seed, List<Translation2d> allObjects) {
        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d other : allObjects) {
            if (isOutOfBounds(other))
                continue;
            if (seed.getDistance(other) <= GamePieceAutoDriveConstants.CLUSTER_RADIUS_METERS)
                neighbors.add(other);
        }
        return neighbors;
    }

    private double calculateClusterScore(int count, double distance) {
        return (count * GamePieceAutoDriveConstants.SCORE_WEIGHT_COUNT)
                - (distance * GamePieceAutoDriveConstants.SCORE_PENALTY_DISTANCE);
    }
}