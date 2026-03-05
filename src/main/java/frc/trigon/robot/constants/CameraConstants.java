package frc.trigon.robot.constants;

import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;

@SuppressWarnings("all")
public class CameraConstants {
    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 1;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            new DynamicCameraTransform((timestampSeconds) -> RobotContainer.INTAKE.calculateIntakeCameraTransformAtTime(timestampSeconds))//IMPORTANT: Leave as lambda expression, method reference will crash code
    );

    private static final StandardDeviations APRIL_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.016,
            0.01
    );
    public static final AprilTagCamera
            RIGHT_TURRET_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "RightTurretCamera",
            new DynamicCameraTransform((timestampSeconds) -> RobotContainer.TURRET.calculateRightCameraTransformAtTime(timestampSeconds)),//IMPORTANT: Leave as lambda expression, method reference will crash code
            APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            LEFT_TURRET_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "LeftTurretCamera",
                    new DynamicCameraTransform((timestampSeconds) -> RobotContainer.TURRET.calculateLeftCameraTransformAtTime(timestampSeconds)),//IMPORTANT: Leave as lambda expression, method reference will crash code
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            );

}