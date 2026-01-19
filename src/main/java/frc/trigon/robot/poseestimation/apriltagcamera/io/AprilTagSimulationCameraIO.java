package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import frc.trigon.robot.poseestimation.apriltagcamera.DynamicCameraTransform;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagSimulationCameraIO extends AprilTagPhotonCameraIO {
    private final PhotonCameraSim cameraSimulation;

    public AprilTagSimulationCameraIO(String cameraName, DynamicCameraTransform dynamicCameraTransform) {
        super(cameraName, dynamicCameraTransform);

        cameraSimulation = new PhotonCameraSim(photonCamera, AprilTagCameraConstants.SIMULATION_CAMERA_PROPERTIES);
        cameraSimulation.enableDrawWireframe(true);
        AprilTagCameraConstants.VISION_SIMULATION.addCamera(cameraSimulation, dynamicCameraTransform.get3dRobotCenterToCamera(Timer.getFPGATimestamp()));
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        AprilTagCameraConstants.VISION_SIMULATION.adjustCamera(cameraSimulation, dynamicCameraTransform.get3dRobotCenterToCamera(Timer.getFPGATimestamp()));
        super.updateInputs(inputs);
    }
}