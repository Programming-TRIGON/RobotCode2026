package frc.trigon.robot.commands.commandclasses.zonerestricteddrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.BoundingBox;

/**
 * A class that represents a zone that the robot is not allowed to leave.
 * Movement toward the zone boundary is slowed and eventually blocked as the robot approaches the edge.
 * Each axis of the zone's coordinate frame is restricted independently, ensuring correct corner behavior.
 */
public class ContainmentZone implements ZoneRestriction {
    private final BoundingBox boundingBox;
    private final double minimumDistanceMeters, brakingZoneDistanceMeters;

    /**
     * Constructs a new ContainmentZone object.
     *
     * @param boundingBox               the bounding box of the containment zone
     * @param minimumDistanceMeters     the distance from the zone boundary that the robot cannot cross
     * @param brakingZoneDistanceMeters the distance from the zone boundary at which braking begins
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
        return calculateRestrictedTranslation(targetTranslation, robotBoundingBox);
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
     * Scales down the components of the translation that point toward the zone boundaries.
     *
     * @param targetTranslation the target translation to restrict
     * @param robotBoundingBox  the robot's current bounding box
     * @return the restricted translation
     */
    private Translation2d calculateRestrictedTranslation(Translation2d targetTranslation, BoundingBox robotBoundingBox) {
        final Pose2d zoneCenter = boundingBox.getCenter();
        final Translation2d
                zoneRelativeTargetTranslation = toZoneRelativeDirection(targetTranslation.unaryMinus(), zoneCenter),
                zoneRelativeRobotCenter = toZoneRelativePosition(robotBoundingBox.getCenter().getTranslation(), zoneCenter);

        final Translation2d zoneRelativeRestrictedTranslation = applyZoneRelativeAxisBraking(zoneRelativeTargetTranslation, zoneRelativeRobotCenter, robotBoundingBox, zoneCenter);
        return fromZoneRelativeDirection(zoneRelativeRestrictedTranslation, zoneCenter).unaryMinus();
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
     * Applies braking independently to each axis of the given zone-relative translation.
     *
     * @param zoneRelativeTargetTranslation the translation in the zone's coordinate frame
     * @param zoneRelativeRobotCenter       the robot's center in the zone's coordinate frame
     * @param robotBoundingBox              the robot's bounding box
     * @param zoneCenter                    the center pose of the containment zone
     * @return the braked translation in the zone's coordinate frame
     */
    private Translation2d applyZoneRelativeAxisBraking(Translation2d zoneRelativeTargetTranslation, Translation2d zoneRelativeRobotCenter, BoundingBox robotBoundingBox, Pose2d zoneCenter) {
        final double
                restrictedXVelocity = applyXAxisBraking(zoneRelativeTargetTranslation.getX(), zoneRelativeRobotCenter.getX(), robotBoundingBox, zoneCenter),
                restrictedYVelocity = applyYAxisBraking(zoneRelativeTargetTranslation.getY(), zoneRelativeRobotCenter.getY(), robotBoundingBox, zoneCenter);
        return new Translation2d(restrictedXVelocity, restrictedYVelocity);
    }

    /**
     * Applies braking to the X velocity component based on the robot's proximity to the zone's X walls.
     *
     * @param xVelocity           the X velocity component in the zone's coordinate frame
     * @param zoneRelativeCenterX the robot's center X coordinate in the zone's coordinate frame
     * @param robotBoundingBox    the robot's bounding box
     * @param zoneCenter          the center pose of the containment zone
     * @return the braked X velocity component
     */
    private double applyXAxisBraking(double xVelocity, double zoneRelativeCenterX, BoundingBox robotBoundingBox, Pose2d zoneCenter) {
        final double
                zoneHalfXWidth = boundingBox.getXWidth() / 2,
                robotHalfXFootprint = calculateRobotXFootprint(robotBoundingBox, zoneCenter) / 2;
        return applyAxisBraking(xVelocity, zoneRelativeCenterX, zoneHalfXWidth, robotHalfXFootprint);
    }

    /**
     * Applies braking to the Y velocity component based on the robot's proximity to the zone's Y walls.
     *
     * @param yVelocity           the Y velocity component in the zone's coordinate frame
     * @param zoneRelativeCenterY the robot's center Y coordinate in the zone's coordinate frame
     * @param robotBoundingBox    the robot's bounding box
     * @param zoneCenter          the center pose of the containment zone
     * @return the braked Y velocity component
     */
    private double applyYAxisBraking(double yVelocity, double zoneRelativeCenterY, BoundingBox robotBoundingBox, Pose2d zoneCenter) {
        final double
                zoneHalfYWidth = boundingBox.getYWidth() / 2,
                robotHalfYFootprint = calculateRobotYFootprint(robotBoundingBox, zoneCenter) / 2;
        return applyAxisBraking(yVelocity, zoneRelativeCenterY, zoneHalfYWidth, robotHalfYFootprint);
    }

    /**
     * Applies braking to a single velocity component based on the robot's position along an axis.
     * Only restricts movement toward a wall that is within the braking zone.
     *
     * @param velocity           the velocity component to restrict
     * @param zoneRelativeCenter the robot's center position along the axis in the zone's coordinate frame
     * @param zoneHalfWidth      the half-width of the zone along the axis
     * @param robotHalfFootprint the robot's projected half-footprint along the axis
     * @return the braked velocity component
     */
    private double applyAxisBraking(double velocity, double zoneRelativeCenter, double zoneHalfWidth, double robotHalfFootprint) {
        final double
                distanceToMaxWall = calculateDistanceToMaxWall(zoneRelativeCenter, zoneHalfWidth, robotHalfFootprint),
                distanceToMinWall = calculateDistanceToMinWall(zoneRelativeCenter, zoneHalfWidth, robotHalfFootprint);

        if (velocity > 0 && distanceToMaxWall < brakingZoneDistanceMeters)
            return velocity * calculateBrakingScale(distanceToMaxWall);
        if (velocity < 0 && distanceToMinWall < brakingZoneDistanceMeters)
            return velocity * calculateBrakingScale(distanceToMinWall);
        return velocity;
    }

    /**
     * Returns the robot's projected half-footprint along the zone's X axis.
     * Accounts for the relative rotation between the robot and the zone.
     *
     * @param robotBoundingBox the robot's bounding box
     * @param zoneCenter       the center pose of the containment zone
     * @return the robot's half-footprint along the zone's X axis
     */
    private static double calculateRobotXFootprint(BoundingBox robotBoundingBox, Pose2d zoneCenter) {
        return calculateRobotFootprintAlongAxis(robotBoundingBox, zoneCenter, true);
    }

    /**
     * Returns the robot's projected half-footprint along the zone's Y axis.
     * Accounts for the relative rotation between the robot and the zone.
     *
     * @param robotBoundingBox the robot's bounding box
     * @param zoneCenter       the center pose of the containment zone
     * @return the robot's half-footprint along the zone's Y axis
     */
    private static double calculateRobotYFootprint(BoundingBox robotBoundingBox, Pose2d zoneCenter) {
        return calculateRobotFootprintAlongAxis(robotBoundingBox, zoneCenter, false);
    }

    private static double calculateRobotFootprintAlongAxis(BoundingBox robotBoundingBox, Pose2d zoneCenter, boolean isXAxis) {
        final Rotation2d zoneRelativeRobotRotation = robotBoundingBox.getRotation().minus(zoneCenter.getRotation());
        final double
                robotXWidthFootprint = robotBoundingBox.getXWidth() * (isXAxis ? Math.abs(zoneRelativeRobotRotation.getCos()) : Math.abs(zoneRelativeRobotRotation.getSin())),
                robotYWidthFootprint = robotBoundingBox.getYWidth() * (isXAxis ? Math.abs(zoneRelativeRobotRotation.getSin()) : Math.abs(zoneRelativeRobotRotation.getCos()));
        return robotXWidthFootprint + robotYWidthFootprint;
    }

    /**
     * Returns the distance from the robot's edge to the zone wall at the maximum extent of the axis.
     *
     * @param zoneRelativeCenter the robot's center position along the axis in the zone's coordinate frame
     * @param zoneHalfWidth      the half-width of the zone along the axis
     * @param robotHalfFootprint the robot's projected half-footprint along the axis
     * @return the distance to the max wall
     */
    private static double calculateDistanceToMaxWall(double zoneRelativeCenter, double zoneHalfWidth, double robotHalfFootprint) {
        return zoneHalfWidth - zoneRelativeCenter - robotHalfFootprint;
    }

    /**
     * Returns the distance from the robot's edge to the zone wall at the minimum extent of the axis.
     *
     * @param zoneRelativeCenter the robot's center position along the axis in the zone's coordinate frame
     * @param zoneHalfWidth      the half-width of the zone along the axis
     * @param robotHalfFootprint the robot's projected half-footprint along the axis
     * @return the distance to the min wall
     */
    private static double calculateDistanceToMinWall(double zoneRelativeCenter, double zoneHalfWidth, double robotHalfFootprint) {
        return zoneRelativeCenter + zoneHalfWidth - robotHalfFootprint;
    }

    /**
     * Converts a field-space position into the zone's coordinate frame.
     *
     * @param position   the field-space position to convert
     * @param zoneCenter the center pose of the containment zone
     * @return the position in the zone's coordinate frame
     */
    private static Translation2d toZoneRelativePosition(Translation2d position, Pose2d zoneCenter) {
        return position.minus(zoneCenter.getTranslation()).rotateBy(zoneCenter.getRotation().unaryMinus());
    }

    /**
     * Converts a field-space direction vector into the zone's coordinate frame.
     *
     * @param direction  the field-space direction to convert
     * @param zoneCenter the center pose of the containment zone
     * @return the direction in the zone's coordinate frame
     */
    private static Translation2d toZoneRelativeDirection(Translation2d direction, Pose2d zoneCenter) {
        return direction.rotateBy(zoneCenter.getRotation().unaryMinus());
    }

    /**
     * Converts a direction vector from the zone's coordinate frame back to field space.
     *
     * @param zoneRelativeDirection the zone-relative direction to convert
     * @param zoneCenter            the center pose of the containment zone
     * @return the direction in field space
     */
    private static Translation2d fromZoneRelativeDirection(Translation2d zoneRelativeDirection, Pose2d zoneCenter) {
        return zoneRelativeDirection.rotateBy(zoneCenter.getRotation());
    }
}