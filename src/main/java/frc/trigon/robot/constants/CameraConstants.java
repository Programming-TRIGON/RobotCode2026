package frc.trigon.robot.constants;

import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;

public class CameraConstants {
    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0.5;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            new DynamicCameraTransform(RobotContainer.INTAKE::calculateIntakeCameraTransformAtTime)
    );

    private static final StandardDeviations APRIL_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.015,
            0.01
    );
    public static final AprilTagCamera
            RIGHT_TURRET_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "RightTurretCamera",
            new DynamicCameraTransform(RobotContainer.TURRET::calculateRightCameraTransformAtTime),
            APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            LEFT_TURRET_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "LeftTurretCamera",
                    new DynamicCameraTransform(RobotContainer.TURRET::calculateLeftCameraTransformAtTime),
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            );

}