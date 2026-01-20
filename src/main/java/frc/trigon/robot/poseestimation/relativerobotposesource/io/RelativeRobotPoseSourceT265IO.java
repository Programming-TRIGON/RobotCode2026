package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.lib.utilities.JsonHandler;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private final NetworkTableEntry jsonDumpEntry;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        jsonDumpEntry = NetworkTableInstance.getDefault().getTable(hostname).getEntry("JsonDump");
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        final T265JsonDump jsonDump = readJsonDump();
        if (jsonDump == null) {
            updateNoResultInputs(inputs);
            return;
        }
        updateHasResultInputs(inputs, jsonDump);
    }

    private void updateNoResultInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        inputs.framesPerSecond = 0;
        inputs.hasResult = false;
    }

    private void updateHasResultInputs(RelativeRobotPoseSourceInputsAutoLogged inputs, T265JsonDump jsonDump) {
        inputs.framesPerSecond = jsonDump.framesPerSecond;
        inputs.batteryPercentage = jsonDump.batteryPercentage;
        inputs.pose = extractPose(jsonDump);
        inputs.resultTimestampSeconds = jsonDumpEntry.getLastChange();
        inputs.hasResult = true;
    }

    private T265JsonDump readJsonDump() {
        return JsonHandler.parseJsonStringToObject(jsonDumpEntry.getString(""), T265JsonDump.class);
    }

    private Pose3d extractPose(T265JsonDump jsonDump) {
        final Translation3d translation = extractTranslation(jsonDump);
        final Rotation3d rotation = extractHeading(jsonDump);
        return new Pose3d(translation, rotation);
    }

    private Translation3d extractTranslation(T265JsonDump jsonDump) {
        return new Translation3d(jsonDump.xPositionMeters, jsonDump.yPositionMeters, jsonDump.zPositionMeters);
    }

    private Rotation3d extractHeading(T265JsonDump jsonDump) {
        return new Rotation3d(0, 0, jsonDump.yawRadians);
    }

    private static class T265JsonDump {
        private int framesPerSecond = 0;
        private double batteryPercentage = 0;
        private double xPositionMeters = 0;
        private double yPositionMeters = 0;
        private double zPositionMeters = 0;
        private double yawRadians = 0;
    }
}