package frc.trigon.robot.misc.objectdetection.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.io.PhotonObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import org.littletonrobotics.junction.Logger;

/**
 * An object detection camera is a class that represents a camera that detects objects other than apriltags, most likely game pieces.
 */
public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private final DynamicCameraTransform dynamicCameraTransform;

    public ObjectDetectionCamera(String hostname, Transform3d robotCenterToCamera) {
        this(hostname, new DynamicCameraTransform(robotCenterToCamera));
    }

    public ObjectDetectionCamera(String hostname, DynamicCameraTransform dynamicCameraTransform) {
        this.hostname = hostname;
        this.dynamicCameraTransform = dynamicCameraTransform;
        this.objectDetectionCameraIO = generateIO(hostname, dynamicCameraTransform);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    /**
     * Calculates the position of the closest object on the field from its 3D rotation relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the closest object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d calculateClosestObjectPositionOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Translation2d[] targetObjectsTranslation = getObjectPositionsOnField(targetGamePiece);
        final Translation2d currentRobotTranslation = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        if (targetObjectsTranslation.length == 0)
            return null;
        Translation2d closestObjectTranslation = targetObjectsTranslation[0];
        double closestObjectDistanceToRobot = currentRobotTranslation.getDistance(closestObjectTranslation);

        for (Translation2d currentObjectTranslation : targetObjectsTranslation) {
            final double currentObjectDistanceToRobot = currentRobotTranslation.getDistance(currentObjectTranslation);
            if (currentObjectDistanceToRobot < closestObjectDistanceToRobot) {
                closestObjectTranslation = currentObjectTranslation;
                closestObjectDistanceToRobot = currentObjectDistanceToRobot;
            }
        }
        return closestObjectTranslation;
    }

    public boolean hasObject(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.hasObject[targetGamePiece.id];
    }

    public Translation2d[] getObjectPositionsOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Rotation3d[] visibleObjectRotations = getObjectsRotations(targetGamePiece);
        final Translation2d[] objectPositionsOnField = new Translation2d[visibleObjectRotations.length];
        final Transform3d robotCenterToCamera = dynamicCameraTransform.get3dRobotCenterToCamera(objectDetectionCameraInputs.latestResultTimestamp);
        final Pose2d robotPoseAtResultTimestamp = RobotContainer.ROBOT_POSE_ESTIMATOR.samplePoseAtTimestamp(objectDetectionCameraInputs.latestResultTimestamp);

        if (robotPoseAtResultTimestamp == null || robotCenterToCamera == null)
            return new Translation2d[0];
        final Pose3d cameraPoseAtTimestamp = new Pose3d(robotPoseAtResultTimestamp).plus(robotCenterToCamera);

        for (int i = 0; i < visibleObjectRotations.length; i++)
            objectPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectRotations[i], cameraPoseAtTimestamp, i);

        Logger.recordOutput("ObjectDetectionCamera/Visible" + targetGamePiece.name(), objectPositionsOnField);
        Logger.recordOutput("CameraaodPose", cameraPoseAtTimestamp);
        return objectPositionsOnField;
    }

    public Rotation3d[] getObjectsRotations(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.visibleObjectRotations[targetGamePiece.id];
    }

    /**
     * Calculates the position of an object on the field from its 3D rotation relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @param objectRotation the object's 3D rotation relative to the camera
     * @return the object's 2D position on the field (z is assumed to be 0)
     */
    private Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation, Pose3d cameraPoseAtTime, int i) {
        final Pose3d objectRotationStart = cameraPoseAtTime.plus(new Transform3d(0, 0, 0, objectRotation));

        final double cameraZ = cameraPoseAtTime.getTranslation().getZ();
        final double zDifference = cameraZ - SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters;
        final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
        final double xTransform = Math.abs(zDifference / objectPitchSin);
        final Transform3d objectRotationStartToGround = new Transform3d(xTransform, 0, 0, new Rotation3d());

        var a = objectRotationStart.transformBy(objectRotationStartToGround).getTranslation();
        Logger.recordOutput("Calc/" + i + "/Height", a.getZ());
        Logger.recordOutput("Calc/" + i + "/X", a.getX());
        Logger.recordOutput("Calc/" + i + "/Yaw", Math.toDegrees(objectRotation.getZ()));
        Logger.recordOutput("Calc/" + i + "/Pitch", Math.toDegrees(objectRotation.getY()));
        return a.toTranslation2d();
    }

    private ObjectDetectionCameraIO generateIO(String hostname, DynamicCameraTransform dynamicCameraTransform) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname, dynamicCameraTransform);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}