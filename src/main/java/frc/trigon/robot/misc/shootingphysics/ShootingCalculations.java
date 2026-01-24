package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    private ShootingState targetShootingState = ShootingState.empty();

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void updateCalculations() {
        targetShootingState = calculateTargetShootingState();
        Logger.recordOutput("Shooting/TargetShootingYawDegrees", targetShootingState.targetFieldRelativeYaw().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingPitchDegrees", targetShootingState.targetPitch().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingVelocityMPS", targetShootingState.targetShootingVelocityMetersPerSecond());
    }

    public ShootingState getTargetShootingState() {
        return targetShootingState;
    }

    @AutoLogOutput(key = "Shooting/CurrentFuelExitPosition")
    public Translation3d calculateCurrentFuelExitPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Rotation2d hoodPitch = RobotContainer.HOOD.getCurrentAngle();
        final Rotation2d turretSelfRelativeYaw = RobotContainer.TURRET.getCurrentSelfRelativeAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, hoodPitch, turretSelfRelativeYaw);
    }

    @AutoLogOutput(key = "Shooting/TargetFuelExitPosition")
    public Translation3d calculateTargetFuelExitPosition() {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.POSE_PREDICTION_TIME_SECONDS);
        final Rotation2d hoodPitch = RobotContainer.HOOD.getTargetAngle();
        final Rotation2d turretSelfRelativeYaw = RobotContainer.TURRET.getTargetSelfRelativeAngle();
        return calculateFieldRelativeFuelExitPose(predictedRobotPose, hoodPitch, turretSelfRelativeYaw);
    }

    public Translation3d calculateFieldRelativeFuelExitPose(Pose2d robotPose, Rotation2d hoodPitch, Rotation2d turretSelfRelativeYaw) {
        final Transform3d robotToFuelExitPosition = calculateRobotToFuelExitTransform(hoodPitch, turretSelfRelativeYaw);
        return new Pose3d(robotPose).transformBy(robotToFuelExitPosition).getTranslation();
    }

    private Transform3d calculateRobotToFuelExitTransform(Rotation2d hoodPitch, Rotation2d turretSelfRelativeYaw) {
        final Pose3d robotRelativeHoodPose = calculateRobotRelativeHoodPose(hoodPitch, turretSelfRelativeYaw);

        final Pose3d fuelExitRobotRelativePosition = robotRelativeHoodPose.transformBy(ShootingCalculationsConstants.HOOD_TO_FUEL_EXIT_POSITION);
        return new Transform3d(
                fuelExitRobotRelativePosition.getTranslation(),
                fuelExitRobotRelativePosition.getRotation()
        );
    }

    private Pose3d calculateRobotRelativeHoodPose(Rotation2d hoodPitch, Rotation2d turretSelfRelativeYaw) {
        final Transform3d turretYawTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, turretSelfRelativeYaw.getRadians())
        );
        final Pose3d robotRelativeTurretPose = ShootingCalculationsConstants.ROBOT_RELATIVE_TURRET_ORIGIN.transformBy(turretYawTransform);
        final Pose3d robotRelativeHoodOrigin = robotRelativeTurretPose.transformBy(ShootingCalculationsConstants.TURRET_TO_HOOD);

        final Transform3d hoodPitchTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -hoodPitch.getRadians(), 0)
        );
        return robotRelativeHoodOrigin.transformBy(hoodPitchTransform);
    }

    private ShootingState calculateTargetShootingState() {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition().toTranslation2d();
        final Translation2d velocity = RobotContainer.SWERVE.getFieldRelativeVelocity();
        final Translation2d hub = FieldConstants.HUB_POSITION.get();

        final double distance = hub.minus(fuelExitPosition).getNorm();
        final Translation2d relativeVelocity = calculateVelocityRelativeToPoint(hub, fuelExitPosition, velocity);
        Logger.recordOutput("Shooting/DistanceToHub", distance);
        Logger.recordOutput("Shooting/RelativeVelocityToHubX", relativeVelocity.getX());
        Logger.recordOutput("Shooting/RelativeVelocityToHubY", relativeVelocity.getY());

        final double targetPitchRadians = ShootingLookupTable3D.calculatePitch(distance, relativeVelocity.getX(), relativeVelocity.getY());
        final double targetShootingVelocityMetersPerSecond = ShootingLookupTable3D.calculateVelocity(distance, relativeVelocity.getX(), relativeVelocity.getY());
        final double yawRadians = ShootingLookupTable3D.calculateYaw(distance, relativeVelocity.getX(), relativeVelocity.getY());
        final Rotation2d targetRobotAngle = calculateAngleToPoint(hub, fuelExitPosition).plus(new Rotation2d(yawRadians));

        return new ShootingState(
                targetRobotAngle,
                new Rotation2d(targetPitchRadians),
                targetShootingVelocityMetersPerSecond
        );
    }

    private Translation3d calculateTangentialRobotVelocityVector(double tangentialVelocityTowardsHub, Rotation2d fuelAngleToHub) {
        final Rotation2d anglePerpendicularToHub = fuelAngleToHub.rotateBy(new Rotation2d(Math.PI / 2));
        return new Translation3d(tangentialVelocityTowardsHub, new Rotation3d(anglePerpendicularToHub));
    }

    private Translation2d calculateVelocityRelativeToPoint(Translation2d fieldPoint, Translation2d currentPosition, Translation2d robotFieldRelativeVelocity) {
        final Rotation2d angleToPoint = calculateAngleToPoint(fieldPoint, currentPosition);
        return robotFieldRelativeVelocity.rotateBy(angleToPoint.unaryMinus());
    }

    private Rotation2d calculateAngleToPoint(Translation2d fieldPoint, Translation2d currentPosition) {
        return fieldPoint.minus(currentPosition).getAngle();
    }

    /**
     * Extracts the yaw of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    public Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(vector.getX(), vector.getY());
    }

    /**
     * Extracts the pitch of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    public Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
    }
}
