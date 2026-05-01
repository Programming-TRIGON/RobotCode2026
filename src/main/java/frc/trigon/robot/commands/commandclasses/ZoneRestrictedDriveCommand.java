package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * Drives the robot while restricting movement relative to defined zones on the field.
 * Restricted zones slow and block movement into them.
 * Containment zones slow and block movement out of them.
 * All zone restrictions are applied sequentially, each further restricting the previous result.
 * Movement parallel to or away from a zone boundary is never restricted.
 */
public class ZoneRestrictedDriveCommand extends ParallelCommandGroup {
    private static final double
            ROBOT_X_WIDTH_METERS = 1,
            ROBOT_Y_WIDTH_METERS = 1;
    private static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.4;
    private static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );

    private final ZoneRestriction[] zoneRestrictions;
    private Translation2d cachedRestrictedTranslation = new Translation2d();

    /**
     * Creates a new ZoneRestrictedDriveCommand.
     *
     * @param shouldRestrictToField whether to restrict the robot from leaving the field boundary
     * @param zoneRestrictions      the zones to restrict movement relative to
     */
    public ZoneRestrictedDriveCommand(boolean shouldRestrictToField, ZoneRestriction... zoneRestrictions) {
        this.zoneRestrictions = buildZoneRestrictions(shouldRestrictToField, zoneRestrictions);
        logAllZoneBoundingBoxes();
        addCommands(
                getTranslationCacheUpdateCommand(),
                getDriveCommand()
        );
    }

    private ZoneRestriction[] buildZoneRestrictions(boolean shouldRestrictToField, ZoneRestriction[] zoneRestrictions) {
        if (!shouldRestrictToField)
            return zoneRestrictions;
        final ZoneRestriction[] allZones = new ZoneRestriction[zoneRestrictions.length + 1];
        allZones[0] = FIELD_BOUNDARY_ZONE;
        System.arraycopy(zoneRestrictions, 0, allZones, 1, zoneRestrictions.length);

        return allZones;
    }

    private void logAllZoneBoundingBoxes() {
        for (int i = 0; i < zoneRestrictions.length; i++)
            zoneRestrictions[i].boundingBox().log("RestrictedZones/" + i);
    }

    private Command getTranslationCacheUpdateCommand() {
        return new RunCommand(() -> cachedRestrictedTranslation = calculateRestrictedTranslation());
    }

    private Command getDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> cachedRestrictedTranslation.getX(),
                () -> cachedRestrictedTranslation.getY(),
                OperatorConstants.DRIVER_CONTROLLER::getRightX
        ).asProxy();
    }

    /**
     * Calculates the target translation after applying all zone restrictions sequentially.
     * Each zone further restricts the translation produced by the previous zone.
     * Input is converted to field coordinates for restriction, then back to joystick coordinates for output.
     *
     * @return the restricted target translation in joystick coordinates
     */
    private Translation2d calculateRestrictedTranslation() {
        final BoundingBox robotBoundingBox = getRobotBoundingBox();

        Translation2d targetTranslation = new Translation2d(
                OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                OperatorConstants.DRIVER_CONTROLLER.getLeftX()
        );

        for (ZoneRestriction zone : zoneRestrictions)
            targetTranslation = applyZoneRestriction(targetTranslation, zone, robotBoundingBox);

        return targetTranslation;
    }

    /**
     * Applies one zone's restriction to the given translation.
     * If the robot is outside the braking zone, the translation is returned unchanged.
     *
     * @param targetTranslation the target translation to restrict
     * @param zone              the zone to apply restriction for
     * @param robotBoundingBox  the robot's current bounding box
     * @return the restricted field-space translation
     */
    private Translation2d applyZoneRestriction(Translation2d targetTranslation, ZoneRestriction zone, BoundingBox robotBoundingBox) {
        final double distanceToBoundary = zone.calculateDistanceToBoundary(robotBoundingBox);

        if (distanceToBoundary > zone.brakingZoneDistanceMeters())
            return targetTranslation;

        return calculateSlowedTranslation(targetTranslation, distanceToBoundary, zone, robotBoundingBox);
    }

    /**
     * Scales down the component of the translation pointing toward the zone boundary.
     * The perpendicular component is left unchanged, allowing the robot to move along the boundary normally.
     * Both the translation and boundary direction are in field coordinates.
     *
     * @param targetTranslation  the target translation to slow
     * @param distanceToBoundary the current distance to the zone boundary
     * @param zone               the zone being applied
     * @param robotBoundingBox   the robot's current bounding box
     * @return the slowed field-space translation
     */
    private Translation2d calculateSlowedTranslation(Translation2d targetTranslation, double distanceToBoundary, ZoneRestriction zone, BoundingBox robotBoundingBox) {
        final Translation2d directionTowardBoundary = calculateUnitVectorTowardBoundary(zone, robotBoundingBox).unaryMinus();
        final double translationComponentTowardBoundary = dotProduct(targetTranslation, directionTowardBoundary);

        if (translationComponentTowardBoundary <= 0)
            return targetTranslation;

        return calculateScaledTranslation(targetTranslation, directionTowardBoundary, translationComponentTowardBoundary, distanceToBoundary, zone);
    }

    /**
     * Splits the translation into a toward-boundary component and a perpendicular component,
     * scales the toward-boundary component by the braking scale, and recombines them.
     *
     * @param fieldSpaceTranslation              the field-space translation to scale
     * @param directionTowardBoundary            the unit vector pointing toward the zone boundary
     * @param translationComponentTowardBoundary the magnitude (-1 to 1) of the translation toward the boundary
     * @param distanceToBoundary                 the current distance to the zone boundary
     * @param zone                               the zone being applied
     * @return the scaled field-space translation
     */
    private Translation2d calculateScaledTranslation(Translation2d fieldSpaceTranslation, Translation2d directionTowardBoundary, double translationComponentTowardBoundary, double distanceToBoundary, ZoneRestriction zone) {
        final double brakingScale = calculateBrakingScale(distanceToBoundary, zone);

        final Translation2d
                towardComponent = directionTowardBoundary.times(translationComponentTowardBoundary),
                perpendicularComponent = fieldSpaceTranslation.minus(towardComponent);

        return perpendicularComponent.plus(towardComponent.times(brakingScale));
    }

    /**
     * Calculates braking strength based off of the distance to the zone boundary.
     * Returns 1 at the outer edge of the braking zone and 0 at the minimum distance.
     *
     * @param distanceToBoundary the current distance to the zone boundary
     * @param zone               the zone being applied
     * @return a scale factor between 0 and 1
     */
    private double calculateBrakingScale(double distanceToBoundary, ZoneRestriction zone) {
        final double
                distanceIntoBrakingZone = distanceToBoundary - zone.minimumDistanceMeters(),
                brakingZoneSize = zone.brakingZoneDistanceMeters() - zone.minimumDistanceMeters();
        return MathUtil.clamp(distanceIntoBrakingZone / brakingZoneSize, 0, 1);
    }

    /**
     * Returns a unit vector pointing from the robot's current position toward the nearest zone boundary point.
     * Returns a zero vector if the robot is exactly on the boundary.
     *
     * @param zone             the zone to calculate the direction toward
     * @param robotBoundingBox the robot's current bounding box
     * @return the unit vector pointing toward the zone boundary, in field coordinates
     */
    private Translation2d calculateUnitVectorTowardBoundary(ZoneRestriction zone, BoundingBox robotBoundingBox) {
        final Translation2d robotCenter = robotBoundingBox.getCenter().getTranslation();
        final Translation2d vectorTowardBoundary = zone.calculateNearestBoundaryPoint(robotCenter).minus(robotCenter);

        if (vectorTowardBoundary.getNorm() < 1e-6)
            return new Translation2d();

        return vectorTowardBoundary.times(1 / vectorTowardBoundary.getNorm());
    }

    /**
     * Returns a bounding box representing the robot's current position and size on the field.
     *
     * @return the robot's current bounding box
     */
    private BoundingBox getRobotBoundingBox() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final BoundingBox robotBoundingBox = new BoundingBox(robotPose, ROBOT_X_WIDTH_METERS, ROBOT_Y_WIDTH_METERS);
        robotBoundingBox.log("RobotBoundingBox");
        return robotBoundingBox;
    }

    private static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    /**
     * Represents a zone that restricts the robot's movement relative to its boundary.
     */
    public sealed interface ZoneRestriction permits RestrictedZone, ContainmentZone {
        BoundingBox boundingBox();

        double minimumDistanceMeters();

        double brakingZoneDistanceMeters();

        /**
         * Returns the distance from the robot to the relevant boundary of this zone.
         *
         * @param robotBoundingBox the robot's current bounding box
         * @return the distance to this zone's boundary
         */
        double calculateDistanceToBoundary(BoundingBox robotBoundingBox);

        /**
         * Returns the nearest point on this zone's relevant boundary to the given position.
         *
         * @param robotCenter the robot's current center position
         * @return the nearest boundary point
         */
        Translation2d calculateNearestBoundaryPoint(Translation2d robotCenter);
    }

    /**
     * A zone that the robot is not allowed to enter.
     * Movement toward the zone is slowed and eventually blocked as the robot approaches.
     *
     * @param boundingBox               the bounding box of the restricted zone
     * @param minimumDistanceMeters     the distance at which movement toward the zone is fully blocked
     * @param brakingZoneDistanceMeters the distance at which braking begins
     */
    public record RestrictedZone(BoundingBox boundingBox, double minimumDistanceMeters,
                                 double brakingZoneDistanceMeters) implements ZoneRestriction {
        @Override
        public double calculateDistanceToBoundary(BoundingBox robotBoundingBox) {
            return robotBoundingBox.distanceTo(boundingBox);
        }

        @Override
        public Translation2d calculateNearestBoundaryPoint(Translation2d robotCenter) {
            return boundingBox.nearest(robotCenter);
        }
    }

    /**
     * A zone that the robot is not allowed to leave.
     * Movement toward the zone boundary is slowed and eventually blocked as the robot approaches the edge.
     *
     * @param boundingBox               the bounding box of the containment zone
     * @param minimumDistanceMeters     the distance from the boundary at which outward movement is fully blocked
     * @param brakingZoneDistanceMeters the distance from the boundary at which braking begins
     */
    public record ContainmentZone(BoundingBox boundingBox, double minimumDistanceMeters,
                                  double brakingZoneDistanceMeters) implements ZoneRestriction {
        @Override
        public double calculateDistanceToBoundary(BoundingBox robotBoundingBox) {
            if (!boundingBox.contains(robotBoundingBox))
                return 0;
            return boundingBox.getMinimumDistanceToPerimeter(robotBoundingBox);
        }

        @Override
        public Translation2d calculateNearestBoundaryPoint(Translation2d robotCenter) {
            var x = boundingBox.nearestPerimeterPoint(robotCenter);
            return boundingBox.contains(robotCenter) ? x : x.unaryMinus();
        }
    }
}