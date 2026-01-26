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
    public Translation3d calculateTargetFuelExitPosition(double posePredictionTimeSeconds) {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(posePredictionTimeSeconds);
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
        final Translation2d robotFieldRelativeVelocity = RobotContainer.SWERVE.getFieldRelativeVelocity();
        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();

        final double targetShootingVelocityMetersPerSecond = calculateTargetShootingVelocityMetersPerSecond(robotFieldRelativeVelocity, hubPosition);
        final Rotation2d targetPitch = calculateTargetPitch(robotFieldRelativeVelocity, hubPosition);
        final Rotation2d targetRobotAngle = calculateTargetRobotAngle(robotFieldRelativeVelocity, hubPosition);

        return new ShootingState(
                targetRobotAngle,
                targetPitch,
                targetShootingVelocityMetersPerSecond
        );
    }

    private double calculateTargetShootingVelocityMetersPerSecond(Translation2d robotFieldRelativeVelocity, Translation2d hubPosition) {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition(ShootingCalculationsConstants.SHOOTER_POSE_PREDICTION_TIME_SECONDS).toTranslation2d();
        final Translation2d robotHubRelativeVelocity = calculateVelocityRelativeToPoint(hubPosition, fuelExitPosition, robotFieldRelativeVelocity);
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();

        return ShootingLookupTable3D.calculateVelocity(
                distanceFromHub,
                robotHubRelativeVelocity.getX(),
                robotHubRelativeVelocity.getY()
        );
    }

    private Rotation2d calculateTargetPitch(Translation2d robotFieldRelativeVelocity, Translation2d hubPosition) {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition(ShootingCalculationsConstants.PITCHER_POSE_PREDICTION_TIME_SECONDS).toTranslation2d();
        final Translation2d robotHubRelativeVelocity = calculateVelocityRelativeToPoint(hubPosition, fuelExitPosition, robotFieldRelativeVelocity);
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();

        Logger.recordOutput("Shooting/DistanceToHub", distanceFromHub);
        Logger.recordOutput("Shooting/HubRelativeVelocityX", robotHubRelativeVelocity.getX());
        Logger.recordOutput("Shooting/HubRelativeVelocityY", robotHubRelativeVelocity.getY());

        return new Rotation2d(ShootingLookupTable3D.calculatePitch(
                distanceFromHub,
                robotHubRelativeVelocity.getX(),
                robotHubRelativeVelocity.getY()
        ));
    }

    private Rotation2d calculateTargetRobotAngle(Translation2d robotFieldRelativeVelocity, Translation2d hubPosition) {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition(ShootingCalculationsConstants.TURRET_POSE_PREDICTION_TIME_SECONDS).toTranslation2d();
        final Translation2d robotHubRelativeVelocity = calculateVelocityRelativeToPoint(hubPosition, fuelExitPosition, robotFieldRelativeVelocity);
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();
        final Rotation2d targetSelfRelativeYaw = new Rotation2d(ShootingLookupTable3D.calculateYaw(
                distanceFromHub,
                robotHubRelativeVelocity.getX(),
                robotHubRelativeVelocity.getY()
        ));

        final Rotation2d robotAngleToHub = calculateAngleToPoint(hubPosition, fuelExitPosition);
        return robotAngleToHub.plus(targetSelfRelativeYaw);
    }

    private Translation2d calculateVelocityRelativeToPoint(Translation2d fieldPoint, Translation2d currentPosition, Translation2d robotFieldRelativeVelocity) {
        final Rotation2d angleToPoint = calculateAngleToPoint(fieldPoint, currentPosition);
        return robotFieldRelativeVelocity.rotateBy(angleToPoint.unaryMinus());
    }

    private Rotation2d calculateAngleToPoint(Translation2d fieldPoint, Translation2d currentPosition) {
        return fieldPoint.minus(currentPosition).getAngle();
    }
}
