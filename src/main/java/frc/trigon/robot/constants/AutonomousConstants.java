package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.LocalADStarAK;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.autonomous.AutonomousGenerator;
import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * A class that contains the constants and configurations for everything related to the 15-second autonomous period at the start of the match.
 */
public class AutonomousConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = 0.7;//TODO: Calibrate
    public static final PathConstraints
            DRIVE_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(4, 7, Units.degreesToRadians(100), Units.degreesToRadians(100)),
            SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_CONSTRAINTS = new PathConstraints(0.2, 0.5, Units.degreesToRadians(100), Units.degreesToRadians(100)),
            SHOOT_PRELOAD_BEFORE_COLLECTING_FROM_DEPOT_CONSTRAINTS = new PathConstraints(1.5, 3.0, Units.degreesToRadians(100), Units.degreesToRadians(100)),
            DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(2.5, 2, Units.degreesToRadians(100), Units.degreesToRadians(100));
    public static final double
            SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_TIME = 1,
            SHOOT_PRELOAD_BEFORE_COLLECTING_FROM_DEPOT_TIME = 2;

    public static double
            TOTAL_MATCH_TIME_SECONDS = 160,
            AUTONOMOUS_TIME_SECONDS = 20,
            DEPOT_COLLECTION_TIMEOUT_SECONDS = 14, //6,
            NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS = 1.5,
            DELIVERY_TIMEOUT_SECONDS = 6,
            SCORING_TIMEOUT_SECONDS = 4,
            ESTIMATED_CLIMBING_TIME_SECONDS = 3,
            START_CLIMBING_TIME_SECONDS = AUTONOMOUS_TIME_SECONDS - ESTIMATED_CLIMBING_TIME_SECONDS;
    public static final double
            ROBOT_AVERAGE_SPEED_METERS_PER_SECOND = 3.0,
            CLIMB_DRIVE_TIME_SAFETY_MARGIN_SECONDS = 0.5;

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(9, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(2, 0, 0) :
                    new PIDConstants(0, 0, 0);

    public static final PIDController GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    public static final PIDController GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(1, 0, 0);
    public static final double AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS = 0.01;

    private static final PPHolonomicDriveController AUTO_PATH_FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS
    );

    /**
     * Initializes PathPlanner. This needs to be called before any PathPlanner function can be used.
     */
    public static void init() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        configureAutoBuilder();
        AutonomousGenerator.init();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose,
                RobotContainer.ROBOT_POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeChassisSpeeds,
                RobotContainer.SWERVE::drivePathPlanner,
                AUTO_PATH_FOLLOWING_CONTROLLER,
                ROBOT_CONFIG,
                Flippable::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}