package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * Restricts the robot's movement when it approaches a defined zone on the field.
 * As the robot enters the braking zone, movement toward the restricted zone is gradually slowed.
 * Movement is fully blocked at the minimum distance. Movement away from or parallel to the zone is never restricted.
 */
public class RestrictMoveIntoZoneCommand extends ParallelCommandGroup {
    private static final double
            ROBOT_X_WIDTH_METERS = 0.8,
            ROBOT_Y_WIDTH_METERS = 0.8;
    private static final double
            MINIMUM_DISTANCE_TO_RESTRICTED_ZONE_METERS = 0.1,
            BRAKING_ZONE_DISTANCE_METERS = 0.5,
            BRAKING_ZONE_SIZE_METERS = BRAKING_ZONE_DISTANCE_METERS - MINIMUM_DISTANCE_TO_RESTRICTED_ZONE_METERS;

    private final BoundingBox restrictedZoneBoundingBox;

    /**
     * Creates a new RestrictMoveIntoZoneCommand.
     *
     * @param restrictedZoneBoundingBox the bounding box of the zone the robot is not allowed to enter
     */
    public RestrictMoveIntoZoneCommand(BoundingBox restrictedZoneBoundingBox) {
        this.restrictedZoneBoundingBox = restrictedZoneBoundingBox;
        addCommands(getRestrictedDriveCommand());
    }

    private Command getRestrictedDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                this::getRestrictedXValue,
                this::getRestrictedYValue,
                OperatorConstants.DRIVER_CONTROLLER::getRightX
        );
    }

    private double getRestrictedXValue() {
        return calculateRestrictedTranslation().getX();
    }

    private double getRestrictedYValue() {
        return calculateRestrictedTranslation().getY();
    }

    /**
     * Calculates the target translation after restricting movement towards the restricted zone.
     * If the robot is outside the braking zone or inside the restricted zone, the raw joystick value is returned unchanged.
     *
     * @return the restricted target translation
     */
    private Translation2d calculateRestrictedTranslation() {
        final Translation2d fieldRelativeJoystickValue = new Translation2d(
                OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                OperatorConstants.DRIVER_CONTROLLER.getLeftX()
        );
        final double distanceToRestrictedZoneMeters = getRobotBoundingBox().distanceTo(restrictedZoneBoundingBox);

        if (distanceToRestrictedZoneMeters == 0 || distanceToRestrictedZoneMeters >= BRAKING_ZONE_DISTANCE_METERS)
            return fieldRelativeJoystickValue;

        return calculateSlowedTranslation(fieldRelativeJoystickValue, distanceToRestrictedZoneMeters);
    }

    /**
     * Scales down the component of the target translation pointing toward the restricted zone.
     * The perpendicular component is left unchanged, allowing the robot to strafe normally.
     *
     * @param fieldRelativeJoystickValue the field relative input from the controller
     * @param distanceToRestrictedZone   the current distance from the robot to the restricted zone
     * @return the slowed translation
     */
    private Translation2d calculateSlowedTranslation(Translation2d fieldRelativeJoystickValue, double distanceToRestrictedZone) {
        final Translation2d unitVectorTowardsRestrictedZone = calculateUnitVectorTowardsRestrictedZone();
        final double translationComponentTowardRestrictedZone = dotProduct(fieldRelativeJoystickValue, unitVectorTowardsRestrictedZone);

        if (translationComponentTowardRestrictedZone <= 0)
            return fieldRelativeJoystickValue;

        return calculateScaledTranslation(fieldRelativeJoystickValue, unitVectorTowardsRestrictedZone, translationComponentTowardRestrictedZone, distanceToRestrictedZone);
    }

    /**
     * Splits the target translation into a toward-zone component and a perpendicular component,
     * scales the toward-zone component by the braking scale, and recombines them.
     *
     * @param fieldRelativeJoystickValue         the field relative input from the controller
     * @param unitVectorTowardsRestrictedZone    the unit vector pointing toward the restricted zone
     * @param inputComponentTowardRestrictedZone the magnitude (-1 to 1) of the input in the toward-zone direction
     * @param distanceToRestrictedZone           the current distance from the robot to the restricted zone
     * @return the scaled translation
     */
    private Translation2d calculateScaledTranslation(Translation2d fieldRelativeJoystickValue, Translation2d unitVectorTowardsRestrictedZone, double inputComponentTowardRestrictedZone, double distanceToRestrictedZone) {
        final double brakingScale = calculateBrakingScale(distanceToRestrictedZone);
        final Translation2d
                towardComponent = unitVectorTowardsRestrictedZone.times(inputComponentTowardRestrictedZone),
                perpendicularComponent = fieldRelativeJoystickValue.minus(towardComponent);

        return perpendicularComponent.plus(towardComponent.times(brakingScale));
    }

    /**
     * Calculates braking strength based off of the distance to the restricted zone.
     * Returns 1 at the outer edge of the braking zone and 0 at the minimum distance.
     *
     * @param distanceToRestrictedZone the current distance from the robot to the restricted zone
     * @return a scale factor between 0 and 1
     */
    private double calculateBrakingScale(double distanceToRestrictedZone) {
        final double distanceIntoBrakingZone = distanceToRestrictedZone - MINIMUM_DISTANCE_TO_RESTRICTED_ZONE_METERS;
        return MathUtil.clamp(
                distanceIntoBrakingZone / BRAKING_ZONE_SIZE_METERS,
                0, 1
        );
    }

    /**
     * Returns a unit vector pointing from the robot's current position toward the nearest point on the restricted zone.
     *
     * @return the unit vector pointing toward the restricted zone
     */
    private Translation2d calculateUnitVectorTowardsRestrictedZone() {
        final Translation2d robotCenter = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d nearestPointOnRestrictedZone = restrictedZoneBoundingBox.nearest(robotCenter);
        final Translation2d vectorTowardRestrictedZone = nearestPointOnRestrictedZone.minus(robotCenter);

        return vectorTowardRestrictedZone.times(1 / vectorTowardRestrictedZone.getNorm());
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

    private static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}