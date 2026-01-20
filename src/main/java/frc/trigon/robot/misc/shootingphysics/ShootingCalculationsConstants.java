package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShootingCalculationsConstants {
    static final Pose3d
            ROBOT_RELATIVE_TURRET_ORIGIN = new Pose3d(
            new Translation3d(-0.14542, 0.14542, 0.34578),
            new Rotation3d(0, 0, 0)
    ),
            ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET = new Pose3d(
                    new Translation3d(-0.06114, 0.14542, 0.46867),
                    new Rotation3d(0, 0, 0)
            );
    static final Transform3d
            TURRET_TO_HOOD = ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET.minus(ROBOT_RELATIVE_TURRET_ORIGIN),
            HOOD_TO_FUEL_EXIT_POSITION = new Transform3d(
                    new Translation3d(-0.1016, 0, 0.0508),
                    new Rotation3d(0, 0, 0)
            );

    static final double POSE_PREDICTION_TIME_SECONDS = 0.2;
}
