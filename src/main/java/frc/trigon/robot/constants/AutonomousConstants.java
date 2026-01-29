package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.LocalADStarAK;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.AutonomousCommands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains the constants and configurations for everything related to the 15-second autonomous period at the start of the match.
 */
public class AutonomousConstants {
    public static final String DEFAULT_AUTO_NAME = "DefaultAutoName";
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = 0.5;//TODO: Calibrate
    public static final PathConstraints
            DRIVE_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(2.5, 4.5, Units.degreesToRadians(450), Units.degreesToRadians(900)),
            DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(1.5, 3.0, Units.degreesToRadians(300), Units.degreesToRadians(600));
    public static LoggedDashboardChooser<Supplier<Command>>
            FIRST_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FirstAutonomousChooser", new SendableChooser<>()),
            SECOND_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SecondAutonomousChooser", new SendableChooser<>()),
            THIRD_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("ThirdAutonomousChooser", new SendableChooser<>());
    public static LoggedDashboardChooser<FlippablePose2d> CLIMB_POSITION_CHOOSER = new LoggedDashboardChooser<>("ClimbChooser", new SendableChooser<>());

    public static double
            DEPOT_COLLECTION_TIMEOUT_SECONDS = 4,
            NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS = 10,
            DELIVERY_TIMEOUT_SECONDS = 10,
            SCORING_TIMEOUT_SECONDS = 6;

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(0, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(0, 0, 0) :
                    new PIDConstants(0, 0, 0);


    public static final PIDController GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    public static final ProfiledPIDController GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    public static final double AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS = 2;

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
        initializeAutoChoosers();

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

    private static void initializeAutoChoosers() {
        configureClimbPositionChooser();
        configureAutonomousPositionChooser(FIRST_AUTONOMOUS_CHOOSER);
        configureAutonomousPositionChooser(SECOND_AUTONOMOUS_CHOOSER);
        configureAutonomousPositionChooser(THIRD_AUTONOMOUS_CHOOSER);
    }

    private static void configureAutonomousPositionChooser(LoggedDashboardChooser<Supplier<Command>> firstAutonomousChooser) {
        firstAutonomousChooser.addOption("Depot", AutonomousCommands::getCollectFromDepotCommand);
        firstAutonomousChooser.addOption("Score", AutonomousCommands::getScoreCommand);
        firstAutonomousChooser.addOption("CollectFromNeutralZone", AutonomousCommands::getCollectFromNeutralZoneCommand);
        firstAutonomousChooser.addOption("Delivery", AutonomousCommands::getDeliveryCommand);
        firstAutonomousChooser.addDefaultOption("Nothing", null);
    }

    private static void configureClimbPositionChooser() {
        CLIMB_POSITION_CHOOSER.addOption("LeftClimb", FieldConstants.LEFT_CLIMB_POSITION);
        CLIMB_POSITION_CHOOSER.addOption("CenterClimb", FieldConstants.CENTER_CLIMB_POSITION);
        CLIMB_POSITION_CHOOSER.addOption("RightClimb", FieldConstants.RIGHT_CLIMB_POSITION);
        CLIMB_POSITION_CHOOSER.addDefaultOption("NoClimb", null);
    }
}