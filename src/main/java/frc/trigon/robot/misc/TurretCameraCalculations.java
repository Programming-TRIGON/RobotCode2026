package frc.trigon.robot.misc;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.RobotContainer;

public class TurretCameraCalculations {
    private static final Transform3d CAMERA_TO_TURRET = new Transform3d();
    private static final Translation3d TURRET_TO_ROBOT = new Translation3d();

    public static Transform3d CAMERA_TO_ROBOT() {
        return CAMERA_TO_TURRET.plus(new Transform3d(TURRET_TO_ROBOT,
                new Rotation3d(RobotContainer.TURRET.getCurrentSelfRelativeAngle())));
    }
}