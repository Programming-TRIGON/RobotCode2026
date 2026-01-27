package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;
import frc.trigon.robot.subsystems.turret.TurretCameraTransformCalculator;

public class CameraConstants {
    private static final Transform3d ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(//TODO: Find actual values
            new Translation3d(0, 0, 0.8),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
    );

    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0.5;
    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );

    private static final StandardDeviations APRIL_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.015,
            0.01
    );
    public static final AprilTagCamera
            RIGHT_TURRET_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "RightTurretCamera",
            new DynamicCameraTransform(TurretCameraTransformCalculator.getInstance()::calculateRobotToRightCameraAtTime),
            APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            LEFT_TURRET_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "LeftTurretCamera",
                    new DynamicCameraTransform(TurretCameraTransformCalculator.getInstance()::calculateRobotToLeftCameraAtTime),
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            );

}