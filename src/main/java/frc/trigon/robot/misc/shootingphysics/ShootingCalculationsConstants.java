package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.*;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.FilesHandler;

public class ShootingCalculationsConstants {
    static final Translation2d TURRET_RADIUS_VECTOR = new Translation2d(-0.1454, 0.1454);
    static final Pose3d
            ROBOT_RELATIVE_TURRET_ORIGIN = new Pose3d(
            new Translation3d(TURRET_RADIUS_VECTOR.getX(), TURRET_RADIUS_VECTOR.getY(), 0.28423),
            new Rotation3d(0, 0, 0)
    ),
            ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET = new Pose3d(
                    new Translation3d(-0.056859, 0.1454, 0.42947),
                    new Rotation3d(0, Math.toRadians(90), 0)
            );
    private static final double X_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS = 0.11942;
    private static final double Z_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS = 0.03955;
    static final Transform3d
            TURRET_TO_HOOD = ROBOT_RELATIVE_HOOD_ORIGIN_AT_ZEROED_TURRET.minus(ROBOT_RELATIVE_TURRET_ORIGIN),
            HOOD_TO_FUEL_EXIT_POSITION = new Transform3d(
                    new Translation3d(-X_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS, 0, Z_FUEL_EXIT_DISTANCE_FROM_HOOD_ORIGIN_METERS),
                    new Rotation3d(0, 0, 0)
            );

    static final double
            HOOD_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0.07,
            SHOOTER_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0.07,
            TURRET_POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.06 : 0.07;

    static final String SHOOTING_LOOKUP_TABLE_FILEPATH = FilesHandler.DEPLOY_PATH + "lut_30_10_veldivflight.bin";
}
