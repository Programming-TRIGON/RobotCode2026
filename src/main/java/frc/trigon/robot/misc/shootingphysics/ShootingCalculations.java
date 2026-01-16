package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.*;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
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
        Logger.recordOutput("Shooting/TargetShootingYawDegrees", targetShootingState.targetFieldRelativeYaw().get().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingPitchDegrees", targetShootingState.targetPitch().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingVelocityMPS", targetShootingState.targetShootingVelocityMetersPerSecond());
    }

    public ShootingState getTargetShootingState() {
        return targetShootingState;
    }

    public Translation3d calculateCurrentFuelExitPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return new Translation3d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), 0.5);

//        return null; // TODO: Implement fuel exit pose calculation
    }

    public Translation2d calculateTargetFuelExitPosition() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return robotPose.getTranslation();
//        return null; // TODO: Implement fuel exit pose calculation
    }

    public Translation3d calculateFuelExitPose(Pose2d robotPose, Rotation2d pitch, Rotation2d turretYaw) {
        return null; // TODO: Implement fuel exit pose calculation
    }

    private ShootingState calculateTargetShootingState() {
        final Translation3d totalShotVector = calculateTotalShotVector();
        final FlippableRotation2d targetRobotAngle = new FlippableRotation2d(getYaw(totalShotVector), false);
        final Rotation2d targetPitch = getPitch(totalShotVector);
        final double targetShootingVelocityMetersPerSecond = totalShotVector.getNorm();
        return new ShootingState(
                targetRobotAngle,
                targetPitch,
                targetShootingVelocityMetersPerSecond
        );
    }

    private Translation3d calculateTotalShotVector() {
        final Translation2d targetFuelExitPosition = calculateTargetFuelExitPosition();
        final Translation2d robotVelocity = RobotContainer.SWERVE.getFieldRelativeVelocity();
        final Translation2d allianceHubPosition = FieldConstants.HUB_POSITION.get();
        final Translation2d hubRelativeRobotVelocity = calculateVelocityRelativeToFieldPoint(allianceHubPosition, targetFuelExitPosition, robotVelocity);
        final Rotation2d fuelAngleToHub = calculateAngleToPoint(allianceHubPosition, targetFuelExitPosition);

        final Translation3d radialShotVector = calculateRadialShotVector(hubRelativeRobotVelocity, fuelAngleToHub, targetFuelExitPosition);
        final Translation3d tangentialRobotVelocityVector = calculateTangentialRobotVelocityVector(hubRelativeRobotVelocity, fuelAngleToHub);
        return radialShotVector.minus(tangentialRobotVelocityVector);
    }

    private Translation3d calculateTangentialRobotVelocityVector(Translation2d hubRelativeRobotVelocity, Rotation2d fuelAngleToHub) {
        final Rotation2d anglePerpendicularToHub = fuelAngleToHub.rotateBy(new Rotation2d(Math.PI / 2));
        return new Translation3d(hubRelativeRobotVelocity.getY(), new Rotation3d(anglePerpendicularToHub));
    }

    private Translation3d calculateRadialShotVector(Translation2d hubRelativeRobotVelocity, Rotation2d fuelAngleToHub, Translation2d fuelExitPosition) {
        final double distanceFromHub = FieldConstants.HUB_POSITION.get().minus(fuelExitPosition).getNorm();
        final double targetShootingSpeedMetersPerSecond = ShootingLookupTable.calculateVelocity(distanceFromHub, hubRelativeRobotVelocity.getX());
        final double targetHoodAngleRadians = ShootingLookupTable.calculatePitch(distanceFromHub, hubRelativeRobotVelocity.getX());
        return new Translation3d(targetShootingSpeedMetersPerSecond, new Rotation3d(0, -targetHoodAngleRadians, fuelAngleToHub.getRadians()));
    }

    private Translation2d calculateVelocityRelativeToFieldPoint(Translation2d fieldPoint, Translation2d currentPosition, Translation2d robotFieldRelativeVelocity) {
        final Rotation2d angleToPoint = calculateAngleToPoint(fieldPoint, currentPosition);
        return robotFieldRelativeVelocity.rotateBy(angleToPoint.unaryMinus());
    }

    private Rotation2d calculateAngleToPoint(Translation2d fieldPoint, Translation2d currentPosition) {
        return fieldPoint.minus(currentPosition).getAngle();
    }

    /**
     * Extracts the yaw off of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    public Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(vector.getX(), vector.getY());
    }

    /**
     * Extracts the pitch off of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    public Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
    }
}
