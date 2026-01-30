package frc.trigon.robot.commands.commandclasses.gamepieceautodrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class GamePieceCluster {
    private final Translation2d centroid;
    private final Rotation2d approachHeading;
    private final double distanceToRobot;

    public GamePieceCluster(List<Translation2d> gamePieces, Pose2d robotPose) {
        this.centroid = calculateCentroid(gamePieces);
        this.distanceToRobot = centroid.getDistance(robotPose.getTranslation());
        this.approachHeading = calculateApproachHeading(gamePieces, robotPose);
    }

    public Translation2d getCentroid() {
        return centroid;
    }

    public Rotation2d getApproachHeading() {
        return approachHeading;
    }

    public double getDistanceToRobot() {
        return distanceToRobot;
    }

    public int getSize() {
        return 1;
    }

    private Translation2d calculateCentroid(List<Translation2d> pieces) {
        double sumX = 0;
        double sumY = 0;
        for (Translation2d piece : pieces) {
            sumX += piece.getX();
            sumY += piece.getY();
        }
        return new Translation2d(sumX / pieces.size(), sumY / pieces.size());
    }

    private Rotation2d calculateApproachHeading(List<Translation2d> pieces, Pose2d robotPose) {
        Rotation2d angleToCentroid = new Rotation2d(
                Math.atan2(centroid.getY() - robotPose.getY(), centroid.getX() - robotPose.getX())
        );

        // If too few points, we cannot determine a line, so face the center
        if (pieces.size() <= 2)
            return angleToCentroid;

        if (isLinear(pieces)) {
            Rotation2d axisAngle = calculatePrincipalAxis(pieces);
            Rotation2d alignedAxis = resolveAmbiguity(axisAngle, angleToCentroid);
            return blendHeadings(angleToCentroid, alignedAxis);
        } else
            return angleToCentroid;
    }

    private boolean isLinear(List<Translation2d> pieces) {
        double varianceX = 0, varianceY = 0, covariance = 0;

        for (Translation2d p : pieces) {
            double dx = p.getX() - centroid.getX();
            double dy = p.getY() - centroid.getY();
            varianceX += dx * dx;
            varianceY += dy * dy;
            covariance += dx * dy;
        }

        double trace = varianceX + varianceY;
        double det = (varianceX * varianceY) - (covariance * covariance);
        double gap = Math.sqrt(Math.max(0, trace * trace - 4 * det));

        double majorEigenvalue = (trace + gap) / 2.0;
        double minorEigenvalue = (trace - gap) / 2.0;

        double aspectRatio = (minorEigenvalue == 0) ? Double.MAX_VALUE : (majorEigenvalue / minorEigenvalue);
        return aspectRatio > GamePieceAutoDriveConstants.LINEARITY_THRESHOLD;
    }

    private Rotation2d calculatePrincipalAxis(List<Translation2d> pieces) {
        double varianceX = 0, varianceY = 0, covariance = 0;

        for (Translation2d p : pieces) {
            double dx = p.getX() - centroid.getX();
            double dy = p.getY() - centroid.getY();
            varianceX += dx * dx;
            varianceY += dy * dy;
            covariance += dx * dy;
        }

        double angleRad = 0.5 * Math.atan2(2 * covariance, varianceX - varianceY);
        return new Rotation2d(angleRad);
    }

    private Rotation2d resolveAmbiguity(Rotation2d axisAngle, Rotation2d angleToCentroid) {
        if (Math.abs(axisAngle.minus(angleToCentroid).getDegrees()) > 90.0) {
            return axisAngle.rotateBy(Rotation2d.fromDegrees(180));
        }
        return axisAngle;
    }

    private Rotation2d blendHeadings(Rotation2d centerHeading, Rotation2d axisHeading) {
        double blendFactor = (GamePieceAutoDriveConstants.BLEND_START_DISTANCE_METERS - distanceToRobot)
                / (GamePieceAutoDriveConstants.BLEND_START_DISTANCE_METERS - GamePieceAutoDriveConstants.BLEND_END_DISTANCE_METERS);

        blendFactor = Math.max(0.0, Math.min(1.0, blendFactor)); // Clamp [0, 1]

        return centerHeading.interpolate(axisHeading, blendFactor);
    }
}