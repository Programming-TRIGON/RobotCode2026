package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.*;

import java.util.function.Function;

public class DynamicCameraTransform {
    private final Function<Double, Transform3d> robotCenterToCameraFunction;

    public DynamicCameraTransform(Transform3d robotCenterToCamera) {
        this(timestamp -> robotCenterToCamera);
    }

    public DynamicCameraTransform(Function<Double, Transform3d> robotCenterToCameraFunction) {
        this.robotCenterToCameraFunction = robotCenterToCameraFunction;
    }

    public Pose2d calculate2dRobotPose(Pose2d cameraPose, double timestampSeconds) {
        final Transform2d robotCenterToCamera = get2dRobotCenterToCamera(timestampSeconds);
        return cameraPose.transformBy(robotCenterToCamera);
    }

    public Pose3d calculate3dRobotPose(Pose3d cameraPose, double timestampSeconds) {
        final Transform3d robotCenterToCamera = get3dRobotCenterToCamera(timestampSeconds);
        return cameraPose.transformBy(robotCenterToCamera);
    }

    public Transform2d get2dRobotCenterToCamera(double timestampSeconds) {
        return toTransform2d(get3dRobotCenterToCamera(timestampSeconds));
    }

    public Transform2d get2dCameraToRobotCenter(double timestampSeconds) {
        return toTransform2d(get3dCameraToRobotCenter(timestampSeconds));
    }

    public Transform3d get3dRobotCenterToCamera(double timestampSeconds) {
        return robotCenterToCameraFunction.apply(timestampSeconds);
    }

    public Transform3d get3dCameraToRobotCenter(double timestampSeconds) {
        return get3dRobotCenterToCamera(timestampSeconds).inverse();
    }

    private Transform2d toTransform2d(Transform3d transform3d) {
        final Translation2d robotCenterToCameraTranslation = transform3d.getTranslation().toTranslation2d();
        final Rotation2d robotCenterToCameraRotation = transform3d.getRotation().toRotation2d();

        return new Transform2d(robotCenterToCameraTranslation, robotCenterToCameraRotation);
    }
}
