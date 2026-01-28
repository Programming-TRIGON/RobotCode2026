package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.FilesHandler;

public class ShootingCalculationsConstants {
    static final Pose3d
            ROBOT_RELATIVE_TURRET_ORIGIN = new Pose3d(
            new Translation3d(-0.14542, 0.14542, 0.34578),
            new Rotation3d(0, 0, 0)
    ),
            ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET = new Pose3d(
                    new Translation3d(-0.06114, 0.14542, 0.46867),
                    new Rotation3d(0, Math.toRadians(90), 0)
            );
    private static final double X_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS = 0.1016;
    private static final double Z_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS = 0.0508;
    static final Transform3d
            TURRET_TO_HOOD = ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET.minus(ROBOT_RELATIVE_TURRET_ORIGIN),
            HOOD_TO_FUEL_EXIT_POSITION = new Transform3d(
                    new Translation3d(-X_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS, 0, Z_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS),
                    new Rotation3d(0, 0, 0)
            );

    static final double
            PITCHER_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0,
            SHOOTER_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0,
            TURRET_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.06 : 0;

    static final String SHOOTING_LOOKUP_TABLE_FILEPATH = FilesHandler.DEPLOY_PATH + "shooting_lut.bin";
}
