package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
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
        this.zoneRestrictions = shouldRestrictToField ? addFieldToZoneRestrictions(zoneRestrictions) : zoneRestrictions;

        addCommands(
                getTranslationCacheUpdateCommand(),
                getDriveCommand()
        );
    }

    private ZoneRestriction[] addFieldToZoneRestrictions(ZoneRestriction[] zoneRestrictions) {
        final ZoneRestriction[] allZones = new ZoneRestriction[zoneRestrictions.length + 1];

        allZones[0] = FIELD_BOUNDARY_ZONE;
        System.arraycopy(zoneRestrictions, 0, allZones, 1, zoneRestrictions.length);

        return allZones;
    }

    private Command getTranslationCacheUpdateCommand() {
        return new RunCommand(() -> cachedRestrictedTranslation = calculateRestrictedTranslation());
    }

    private Command getDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> cachedRestrictedTranslation.getX(),
                () -> cachedRestrictedTranslation.getY(),
                () -> CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX())
        ).asProxy();
    }

    /**
     * Calculates the target translation after applying all zone restrictions sequentially.
     * Each zone further restricts the translation produced by the previous zone.
     *
     * @return the restricted target translation
     */
    private Translation2d calculateRestrictedTranslation() {
        final BoundingBox robotBoundingBox = getRobotBoundingBox();
        Translation2d targetTranslation = new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX())
        );

        for (ZoneRestriction zone : zoneRestrictions)
            targetTranslation = zone.applyRestriction(targetTranslation, robotBoundingBox);

        return targetTranslation;
    }

    /**
     * Returns a bounding box representing the robot's current position and size on the field.
     *
     * @return the robot's current bounding box
     */
    private BoundingBox getRobotBoundingBox() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return new BoundingBox(robotPose, ROBOT_X_WIDTH_METERS, ROBOT_Y_WIDTH_METERS);
    }

    /**
     * Represents a zone that restricts the robot's movement relative to its boundary.
     */
    public sealed interface ZoneRestriction permits RestrictedZone, ContainmentZone {
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
        public RestrictedZone {
            if (minimumDistanceMeters < 0 || brakingZoneDistanceMeters < 0)
                DriverStation.reportWarning("RestrictedZone distances must be non-negative", false);
            if (minimumDistanceMeters > brakingZoneDistanceMeters)
                DriverStation.reportWarning("RestrictedZone minimum distance cannot be greater than braking zone distance", false);

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

    /**
     * A zone that the robot is not allowed to leave.
     * Movement toward the zone boundary is slowed and eventually blocked as the robot approaches the edge.
     * Each axis of the zone's local frame is restricted independently, ensuring correct corner behavior.
     * Wall distances account for the robot's physical size, so braking begins relative to the robot's edge.
     *
     * @param boundingBox               the bounding box of the containment zone
     * @param minimumDistanceMeters     the distance from the boundary at which outward movement is fully blocked
     * @param brakingZoneDistanceMeters the distance from the boundary at which braking begins
     */
    public record ContainmentZone(BoundingBox boundingBox, double minimumDistanceMeters,
                                  double brakingZoneDistanceMeters) implements ZoneRestriction {
        public ContainmentZone {
            if (minimumDistanceMeters < 0 || brakingZoneDistanceMeters < 0)
                DriverStation.reportWarning("ContainmentZone distances must be non-negative", false);
            if (minimumDistanceMeters > brakingZoneDistanceMeters)
                DriverStation.reportWarning("ContainmentZone minimum distance cannot be greater than braking zone distance", false);
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
}