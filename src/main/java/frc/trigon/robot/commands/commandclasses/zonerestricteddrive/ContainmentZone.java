package frc.trigon.robot.commands.commandclasses.zonerestricteddrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.BoundingBox;

/**
 * A class that represents a zone that the robot is not allowed to leave.
 * Movement toward the zone boundary is slowed and eventually blocked as the robot approaches the edge.
 * Each axis of the zone's local frame is restricted independently, ensuring correct corner behavior.
 */
public class ContainmentZone implements ZoneRestriction {
    private final BoundingBox boundingBox;
    private final double minimumDistanceMeters, brakingZoneDistanceMeters;

    /**
     * Constructs a new ContainmentZone object.
     *
     * @param boundingBox               the bounding box of the containment zone
     * @param minimumDistanceMeters     the distance from the boundary at which outward movement is fully blocked
     * @param brakingZoneDistanceMeters the distance from the boundary at which braking begins
     */
    public ContainmentZone(BoundingBox boundingBox, double minimumDistanceMeters, double brakingZoneDistanceMeters) {
        this.boundingBox = boundingBox;
        this.minimumDistanceMeters = minimumDistanceMeters;
        this.brakingZoneDistanceMeters = brakingZoneDistanceMeters;

        if (minimumDistanceMeters < 0 || brakingZoneDistanceMeters < 0)
            DriverStation.reportWarning("ContainmentZone distances must be non-negative", false);
        if (minimumDistanceMeters > brakingZoneDistanceMeters)
            DriverStation.reportWarning("ContainmentZone minimum distance cannot be greater than braking zone distance", false);

    }

    @Override
    public double minimumDistanceMeters() {
        return minimumDistanceMeters;
    }

    @Override
    public double brakingZoneDistanceMeters() {
        return brakingZoneDistanceMeters;
    }

    @Override
    public Translation2d applyRestriction(Translation2d targetTranslation, BoundingBox robotBoundingBox) {
        if (!isWithinBrakingZone(robotBoundingBox))
            return targetTranslation;
        return calculateBrakedTranslation(targetTranslation, robotBoundingBox);
    }

    /**
     * Computes the braked translation by restricting each local axis independently.
     *
     * @param targetTranslation the translation to slow down
     * @param robotBoundingBox  the robot's current bounding box
     * @return the braked translation
     */
    private Translation2d calculateBrakedTranslation(Translation2d targetTranslation, BoundingBox robotBoundingBox) {
        final Pose2d boxCenter = boundingBox.getCenter();
        final Translation2d localRobotCenter = toLocalPosition(robotBoundingBox.getCenter().getTranslation(), boxCenter);
        final Translation2d localTranslation = toLocalDirection(targetTranslation.unaryMinus(), boxCenter);

        final double robotProjectedHalfX = calculateRobotProjectedHalfExtentX(robotBoundingBox, boxCenter);
        final double robotProjectedHalfY = calculateRobotProjectedHalfExtentY(robotBoundingBox, boxCenter);
        final double halfXWidth = boundingBox.getXWidth() / 2.0;
        final double halfYWidth = boundingBox.getYWidth() / 2.0;

        final double restrictedLocalVx = applyAxisBraking(
                localTranslation.getX(),
                halfXWidth - localRobotCenter.getX() - robotProjectedHalfX,
                localRobotCenter.getX() + halfXWidth - robotProjectedHalfX
        );
        final double restrictedLocalVy = applyAxisBraking(
                localTranslation.getY(),
                halfYWidth - localRobotCenter.getY() - robotProjectedHalfY,
                localRobotCenter.getY() + halfYWidth - robotProjectedHalfY
        );

        return fromLocalDirection(new Translation2d(restrictedLocalVx, restrictedLocalVy), boxCenter).unaryMinus();
    }

    /**
     * Applies braking to a single velocity component based on its distance to the walls on each side.
     * Only restricts movement toward a wall that is within the braking zone.
     *
     * @param velocity               the velocity component to restrict
     * @param distanceToPositiveWall the distance from the robot's edge to the wall in the positive direction
     * @param distanceToNegativeWall the distance from the robot's edge to the wall in the negative direction
     * @return the braked velocity component
     */
    private double applyAxisBraking(double velocity, double distanceToPositiveWall, double distanceToNegativeWall) {
        if (velocity > 0 && distanceToPositiveWall < brakingZoneDistanceMeters)
            return velocity * calculateBrakingScale(distanceToPositiveWall);
        if (velocity < 0 && distanceToNegativeWall < brakingZoneDistanceMeters)
            return velocity * calculateBrakingScale(distanceToNegativeWall);
        return velocity;
    }

    /**
     * Returns whether the robot is close enough to the boundary for braking to apply.
     *
     * @param robotBoundingBox the robot's current bounding box
     * @return whether the robot is within the braking zone
     */
    private boolean isWithinBrakingZone(BoundingBox robotBoundingBox) {
        return calculateDistanceToBoundary(robotBoundingBox) <= brakingZoneDistanceMeters;
    }

    /**
     * Returns the distance from the robot's bounding box to the nearest point on the containment zone's perimeter.
     * Returns 0 if the robot is not fully contained within the zone.
     *
     * @param robotBoundingBox the robot's current bounding box
     * @return the distance to the nearest perimeter point
     */
    private double calculateDistanceToBoundary(BoundingBox robotBoundingBox) {
        if (!boundingBox.contains(robotBoundingBox))
            return 0;
        return boundingBox.getMinimumDistanceToPerimeter(robotBoundingBox);
    }

    /**
     * Returns how far the robot extends from its center along the zone's local X axis.
     * Accounts for the relative rotation between the robot and the zone.
     *
     * @param robotBoundingBox the robot's bounding box
     * @param boxCenter        the center pose of the containment zone
     * @return the projected half-extent of the robot along the zone's local X axis
     */
    private static double calculateRobotProjectedHalfExtentX(BoundingBox robotBoundingBox, Pose2d boxCenter) {
        final Rotation2d relativeRotation = robotBoundingBox.getRotation().minus(boxCenter.getRotation());
        return robotBoundingBox.getXWidth() / 2.0 * Math.abs(relativeRotation.getCos())
                + robotBoundingBox.getYWidth() / 2.0 * Math.abs(relativeRotation.getSin());
    }

    /**
     * Returns how far the robot extends from its center along the zone's local Y axis.
     * Accounts for the relative rotation between the robot and the zone.
     *
     * @param robotBoundingBox the robot's bounding box
     * @param boxCenter        the center pose of the containment zone
     * @return the projected half-extent of the robot along the zone's local Y axis
     */
    private static double calculateRobotProjectedHalfExtentY(BoundingBox robotBoundingBox, Pose2d boxCenter) {
        final Rotation2d relativeRotation = robotBoundingBox.getRotation().minus(boxCenter.getRotation());
        return robotBoundingBox.getXWidth() / 2.0 * Math.abs(relativeRotation.getSin())
                + robotBoundingBox.getYWidth() / 2.0 * Math.abs(relativeRotation.getCos());
    }

    /**
     * Converts a field-space position into the zone's local coordinate frame.
     *
     * @param position  the field-space position to convert
     * @param boxCenter the center pose of the containment zone
     * @return the position in the zone's local frame
     */
    private static Translation2d toLocalPosition(Translation2d position, Pose2d boxCenter) {
        return position.minus(boxCenter.getTranslation()).rotateBy(boxCenter.getRotation().unaryMinus());
    }

    /**
     * Converts a field-space direction vector into the zone's local coordinate frame.
     *
     * @param direction the field-space direction to convert
     * @param boxCenter the center pose of the containment zone
     * @return the direction in the zone's local frame
     */
    private static Translation2d toLocalDirection(Translation2d direction, Pose2d boxCenter) {
        return direction.rotateBy(boxCenter.getRotation().unaryMinus());
    }

    /**
     * Converts a direction vector from the zone's local coordinate frame back to field space.
     *
     * @param localDirection the local-frame direction to convert
     * @param boxCenter      the center pose of the containment zone
     * @return the direction in field space
     */
    private static Translation2d fromLocalDirection(Translation2d localDirection, Pose2d boxCenter) {
        return localDirection.rotateBy(boxCenter.getRotation());
    }
}
