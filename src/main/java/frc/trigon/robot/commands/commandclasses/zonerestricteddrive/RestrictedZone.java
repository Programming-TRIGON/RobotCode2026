package frc.trigon.robot.commands.commandclasses.zonerestricteddrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.BoundingBox;

/**
 * A class that represents a zone that the robot is not allowed to enter.
 * Movement toward the zone is slowed and eventually blocked as the robot approaches.
 */
public class RestrictedZone implements ZoneRestriction {
    private final BoundingBox boundingBox;
    private final double minimumDistanceMeters, brakingZoneDistanceMeters;

    /**
     * Constructs a new RestrictedZone object.
     *
     * @param boundingBox               the bounding box of the restricted zone
     * @param minimumDistanceMeters     the distance at which movement toward the zone is fully blocked
     * @param brakingZoneDistanceMeters the distance at which braking begins
     */
    public RestrictedZone(BoundingBox boundingBox, double minimumDistanceMeters, double brakingZoneDistanceMeters) {
        this.boundingBox = boundingBox;
        this.minimumDistanceMeters = minimumDistanceMeters;
        this.brakingZoneDistanceMeters = brakingZoneDistanceMeters;

        if (minimumDistanceMeters < 0 || brakingZoneDistanceMeters < 0)
            DriverStation.reportWarning("RestrictedZone distances must be non-negative", false);
        if (minimumDistanceMeters > brakingZoneDistanceMeters)
            DriverStation.reportWarning("RestrictedZone minimum distance cannot be greater than braking zone distance", false);
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
        final double distanceToBoundary = robotBoundingBox.distanceTo(boundingBox);
        if (distanceToBoundary > brakingZoneDistanceMeters)
            return targetTranslation;
        return calculateBrakedTranslation(targetTranslation, distanceToBoundary, robotBoundingBox);
    }

    /**
     * Scales down the component of the translation pointing toward the zone boundary.
     *
     * @param targetTranslation  the translation to slow down
     * @param distanceToBoundary the current distance to the zone boundary
     * @param robotBoundingBox   the robot's current bounding box
     * @return the slowed translation
     */
    private Translation2d calculateBrakedTranslation(Translation2d targetTranslation, double distanceToBoundary, BoundingBox robotBoundingBox) {
        final Translation2d directionTowardBoundary = calculateDirectionTowardBoundary(robotBoundingBox).unaryMinus();
        final double translationComponentTowardBoundary = dotProduct(targetTranslation, directionTowardBoundary);

        if (translationComponentTowardBoundary <= 0)
            return targetTranslation;

        return applyBrakingScale(targetTranslation, directionTowardBoundary, translationComponentTowardBoundary, calculateBrakingScale(distanceToBoundary));
    }

    /**
     * Returns a unit vector pointing from the robot's current position toward the nearest zone boundary point.
     * If the robot is inside the zone, the vector is flipped to point toward the boundary from inside.
     * Returns a zero vector if the robot is exactly on the boundary.
     *
     * @param robotBoundingBox the robot's current bounding box
     * @return the unit vector pointing toward the zone boundary
     */
    private Translation2d calculateDirectionTowardBoundary(BoundingBox robotBoundingBox) {
        final Translation2d robotCenter = robotBoundingBox.getCenter().getTranslation();
        Translation2d vectorTowardBoundary = boundingBox.nearestPerimeterPoint(robotCenter).minus(robotCenter);

        if (boundingBox.contains(robotCenter))
            vectorTowardBoundary = vectorTowardBoundary.unaryMinus();

        if (vectorTowardBoundary.getNorm() < 1e-6)
            return new Translation2d();

        return vectorTowardBoundary.times(1 / vectorTowardBoundary.getNorm());
    }

    /**
     * Splits the translation into a toward-boundary component and a perpendicular component,
     * scales the toward-boundary component by the braking scale, and recombines them.
     *
     * @param translation        the translation to scale
     * @param direction          the unit vector pointing toward the boundary
     * @param componentMagnitude the magnitude of the translation in the toward-boundary direction
     * @param brakingScale       the scale factor to apply to the toward-boundary component
     * @return the scaled translation
     */
    private static Translation2d applyBrakingScale(Translation2d translation, Translation2d direction, double componentMagnitude, double brakingScale) {
        final Translation2d
                towardComponent = direction.times(componentMagnitude),
                perpendicularComponent = translation.minus(towardComponent);
        return perpendicularComponent.plus(towardComponent.times(brakingScale));
    }

    private static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}
