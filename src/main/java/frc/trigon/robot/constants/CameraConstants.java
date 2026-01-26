package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.TurretCameraCalculations;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;

public class CameraConstants {
    private static final Transform3d ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(//TODO: AHAHAHAHAH
            new Translation3d(0, 0, 0.8),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
    );

    private static final DynamicCameraTransform ROBOT_CENTER_TO_HUB_TAG_CAMERA = new DynamicCameraTransform(timeStamp -> TurretCameraCalculations.CAMERA_TO_ROBOT());

    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0.5;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );

    public static final AprilTagCamera HUB_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "HubTagCamera", ROBOT_CENTER_TO_HUB_TAG_CAMERA,
            new StandardDeviations(1, 1));
}