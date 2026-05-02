package frc.trigon.robot.commands.commandclasses.zonerestricteddrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;

/**
 * Represents a zone that restricts the robot's movement relative to its boundary.
 */
public interface ZoneRestriction {
    double minimumDistanceMeters();

    double brakingZoneDistanceMeters();

    /**
     * Applies this zone's movement restriction to the given translation.
     *
     * @param targetTranslation the current target translation in joystick coordinates
     * @param robotBoundingBox  the robot's current bounding box
     * @return the restricted target translation
     */
    Translation2d applyRestriction(Translation2d targetTranslation, BoundingBox robotBoundingBox);

    /**
     * Calculates braking strength based on the distance to the zone boundary.
     * Returns 1 at the outer edge of the braking zone and 0 at the minimum distance.
     *
     * @param distanceToBoundary the current distance to the zone boundary
     * @return a scale factor between 0 and 1
     */
    default double calculateBrakingScale(double distanceToBoundary) {
        final double
                distanceIntoBrakingZone = distanceToBoundary - minimumDistanceMeters(),
                brakingZoneSize = brakingZoneDistanceMeters() - minimumDistanceMeters();
        return MathUtil.clamp(distanceIntoBrakingZone / brakingZoneSize, 0, 1);
    }
}
