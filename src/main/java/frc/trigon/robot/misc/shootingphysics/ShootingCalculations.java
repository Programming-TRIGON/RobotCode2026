package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public Translation3d calculateTargetFuelExitPosition(Pose2d robotPose) {
        final Rotation2d hoodPitch = RobotContainer.HOOD.getTargetAngle();
        final Rotation2d turretSelfRelativeYaw = RobotContainer.TURRET.getTargetSelfRelativeAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, hoodPitch, turretSelfRelativeYaw);
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
        final ChassisSpeeds fieldRelativeChassisSpeeds = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds();
        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();

        final double targetShootingVelocityMetersPerSecond = calculateTargetShootingVelocityMetersPerSecond(fieldRelativeChassisSpeeds, hubPosition);
        final Rotation2d targetPitch = calculateTargetPitch(fieldRelativeChassisSpeeds, hubPosition);
        final Rotation2d targetRobotAngle = calculateTargetRobotAngle(fieldRelativeChassisSpeeds, hubPosition);

        return new ShootingState(
                targetRobotAngle,
                targetPitch,
                targetShootingVelocityMetersPerSecond
        );
    }

    public ShootingState calculateTargetShootingState(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {
        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();

        final double targetShootingVelocityMetersPerSecond = calculateTargetShootingVelocityMetersPerSecond(robotPose, fieldRelativeChassisSpeeds, hubPosition);
        final Rotation2d targetPitch = calculateTargetPitch(robotPose, fieldRelativeChassisSpeeds, hubPosition);
        final Rotation2d targetRobotAngle = calculateTargetRobotAngle(robotPose, fieldRelativeChassisSpeeds, hubPosition);

        return new ShootingState(
                targetRobotAngle,
                targetPitch,
                targetShootingVelocityMetersPerSecond
        );
    }

    private double calculateTargetShootingVelocityMetersPerSecond(ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.SHOOTER_POSE_PREDICTION_TIME_SECONDS);
        return calculateTargetShootingVelocityMetersPerSecond(predictedRobotPose, fieldRelativeChassisSpeeds, hubPosition);
    }

    private double calculateTargetShootingVelocityMetersPerSecond(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition(robotPose).toTranslation2d();
        final Translation2d hubRelativeFuelVelocity = calculateHubRelativeFuelVelocity(hubPosition, fuelExitPosition, fieldRelativeChassisSpeeds, robotPose.getRotation());
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();

        return ShootingLookupTable3D.calculateVelocity(
                distanceFromHub,
                hubRelativeFuelVelocity.getX(),
                hubRelativeFuelVelocity.getY()
        );
    }

    private Rotation2d calculateTargetPitch(ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.HOOD_POSE_PREDICTION_TIME_SECONDS);
        return calculateTargetPitch(predictedRobotPose, fieldRelativeChassisSpeeds, hubPosition);
    }

    private Rotation2d calculateTargetPitch(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Translation3d fuelExitPosition3d = calculateTargetFuelExitPosition(robotPose);
        final Translation2d fuelExitPosition = fuelExitPosition3d.toTranslation2d();
        final Translation2d hubRelativeFuelVelocity = calculateHubRelativeFuelVelocity(hubPosition, fuelExitPosition, fieldRelativeChassisSpeeds, robotPose.getRotation());
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();

        Logger.recordOutput("Shooting/DistanceToHub", distanceFromHub);
        Logger.recordOutput("Shooting/TargetFuelExitPosition", new Pose3d(fuelExitPosition3d, new Rotation3d()));
        Logger.recordOutput("Shooting/HubRelativeVelocityX", hubRelativeFuelVelocity.getX());
        Logger.recordOutput("Shooting/HubRelativeVelocityY", hubRelativeFuelVelocity.getY());

        return new Rotation2d(ShootingLookupTable3D.calculatePitch(
                distanceFromHub,
                hubRelativeFuelVelocity.getX(),
                hubRelativeFuelVelocity.getY()
        ));
    }

    private Rotation2d calculateTargetRobotAngle(ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.TURRET_POSE_PREDICTION_TIME_SECONDS);
        return calculateTargetRobotAngle(predictedRobotPose, fieldRelativeChassisSpeeds, hubPosition);
    }

    private Rotation2d calculateTargetRobotAngle(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d hubPosition) {
        final Translation2d fuelExitPosition = calculateTargetFuelExitPosition(robotPose).toTranslation2d();
        final Translation2d hubRelativeFuelVelocity = calculateHubRelativeFuelVelocity(hubPosition, fuelExitPosition, fieldRelativeChassisSpeeds, robotPose.getRotation());
        final double distanceFromHub = hubPosition.minus(fuelExitPosition).getNorm();
        final Rotation2d targetSelfRelativeYaw = new Rotation2d(ShootingLookupTable3D.calculateYaw(
                distanceFromHub,
                hubRelativeFuelVelocity.getX(),
                hubRelativeFuelVelocity.getY()
        ));

        final Rotation2d turretAngleToHub = calculateAngleToPoint(hubPosition, fuelExitPosition);
        return turretAngleToHub.plus(targetSelfRelativeYaw);
    }

    public static Translation2d calculateHubRelativeFuelVelocity(Translation2d hubPosition, Translation2d turretPosition, ChassisSpeeds fieldRelativeChassisSpeeds, Rotation2d robotFieldRelativeAngle) {
        final Translation2d fuelHubRelativeVelocity = calculateVelocityRelativeToPoint(hubPosition, turretPosition, new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond));
        final Translation2d fieldRelativeTurretOffset = ShootingCalculationsConstants.TURRET_RADIUS_VECTOR.rotateBy(robotFieldRelativeAngle);
        final Translation2d fieldRelativeRotationalLinearVelocity = new Translation2d(
                -fieldRelativeChassisSpeeds.omegaRadiansPerSecond * fieldRelativeTurretOffset.getY(),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond * fieldRelativeTurretOffset.getX()
        );
        final Translation2d hubRelativeRotationalLinearVelocity = calculateVelocityRelativeToPoint(hubPosition, turretPosition, fieldRelativeRotationalLinearVelocity);

        return fuelHubRelativeVelocity.plus(hubRelativeRotationalLinearVelocity);

    }

    public static Translation2d calculateVelocityRelativeToPoint(Translation2d fieldPoint, Translation2d currentPosition, Translation2d robotFieldRelativeVelocity) {
        final Rotation2d angleToPoint = calculateAngleToPoint(fieldPoint, currentPosition);
        return robotFieldRelativeVelocity.rotateBy(angleToPoint.unaryMinus());
    }

    public static Rotation2d calculateAngleToPoint(Translation2d fieldPoint, Translation2d currentPosition) {
        return fieldPoint.minus(currentPosition).getAngle();
    }
}
