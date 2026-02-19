package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.MatchTracker;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class AutonomousGenerator {
    public static final LoggedDashboardChooser<AutonomousState>
            FIRST_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FirstAutonomousChooser", new SendableChooser<>()),
            SECOND_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SecondAutonomousChooser", new SendableChooser<>()),
            THIRD_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("ThirdAutonomousChooser", new SendableChooser<>()),
            FOURTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FourthAutonomousChooser", new SendableChooser<>()),
            FIFTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FifthAutonomousChooser", new SendableChooser<>());
    public static final LoggedDashboardChooser<AutonomousClimbPosition> CLIMB_POSITION_CHOOSER = new LoggedDashboardChooser<>("AutonomousClimbChooser", new SendableChooser<>());
    public static final LoggedNetworkBoolean IS_AUTONOMOUS_CLIMB_HIGHEST_PRIORITY = new LoggedNetworkBoolean("IsClimbHighestPriority", true);

    public static void init() {
        configureAutonomousChooser(FIRST_AUTONOMOUS_CHOOSER);
        configureAutonomousChooser(SECOND_AUTONOMOUS_CHOOSER);
        configureAutonomousChooser(THIRD_AUTONOMOUS_CHOOSER);
        configureAutonomousChooser(FOURTH_AUTONOMOUS_CHOOSER);
        configureAutonomousChooser(FIFTH_AUTONOMOUS_CHOOSER);
        configureClimbPositionChooser(CLIMB_POSITION_CHOOSER);
    }

    public static Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                getAutonomousStateSequenceCommand().until(AutonomousGenerator::shouldStartDrivingToClimb),
                GeneralAutonomousCommands.getClimbCommand(() -> CLIMB_POSITION_CHOOSER.get().climbPose).onlyIf(AutonomousGenerator::shouldClimb),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)
        ).alongWith(getLogCommand());
    }

    private static Command getAutonomousStateSequenceCommand() {
        return new SequentialCommandGroup(
                getCommandFromState(FIRST_AUTONOMOUS_CHOOSER.get(), null, SECOND_AUTONOMOUS_CHOOSER.get()),
                getCommandFromState(SECOND_AUTONOMOUS_CHOOSER.get(), FIRST_AUTONOMOUS_CHOOSER.get(), THIRD_AUTONOMOUS_CHOOSER.get()),
                getCommandFromState(THIRD_AUTONOMOUS_CHOOSER.get(), SECOND_AUTONOMOUS_CHOOSER.get(), FOURTH_AUTONOMOUS_CHOOSER.get()),
                getCommandFromState(FOURTH_AUTONOMOUS_CHOOSER.get(), THIRD_AUTONOMOUS_CHOOSER.get(), FIFTH_AUTONOMOUS_CHOOSER.get()),
                getCommandFromState(FIFTH_AUTONOMOUS_CHOOSER.get(), FOURTH_AUTONOMOUS_CHOOSER.get(), null)
        );
    }

    private static Command getCommandFromState(AutonomousState state, AutonomousState previousState, AutonomousState nextState) {
        if (state == null)
            return Commands.none();

        return switch (state) {
            case DELIVERY ->
                    GeneralAutonomousCommands.getDeliveryCommand(previousState, AutonomousConstants.DELIVERY_TIMEOUT_SECONDS::get);
            case SCORE ->
                    GeneralAutonomousCommands.getScoreCommand(nextState, AutonomousConstants.SCORING_TIMEOUT_SECONDS);
            case COLLECT_FROM_DEPOT ->
                    GeneralAutonomousCommands.getCollectFromDepotCommand(true, AutonomousConstants.DEPOT_COLLECTION_TIMEOUT_SECONDS);
            case COLLECT_FROM_NEUTRAL_ZONE ->
                    GeneralAutonomousCommands.getCollectFromNeutralZoneCommand(previousState, AutonomousConstants.NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS);
        };
    }

    private static Command getLogCommand() {
        return new RunCommand(AutonomousGenerator::log);
    }

    private static void log() {
        Logger.recordOutput("Autonomous/ShouldClimb", shouldClimb());
        Logger.recordOutput("Autonomous/ShouldStartDrivingToClimb", shouldStartDrivingToClimb());
        Logger.recordOutput("Autonomous/IsRight", SafeAutonomousDriveCommands.isRight());
        Logger.recordOutput("Autonomous/IsInAllianceZone", SafeAutonomousDriveCommands.isInAllianceZone());
    }

    private static boolean shouldStartDrivingToClimb() {
        if (!shouldClimb() || !IS_AUTONOMOUS_CLIMB_HIGHEST_PRIORITY.get())
            return false;

        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose2d targetClimbPose = CLIMB_POSITION_CHOOSER.get().climbPose.get();
        final double distanceToClimbPoseMeters = currentRobotPose.getTranslation().getDistance(targetClimbPose.getTranslation());
        final double estimatedDriveTimeSeconds = distanceToClimbPoseMeters / AutonomousConstants.ROBOT_AVERAGE_SPEED_METERS_PER_SECOND;
        final double timeToLeaveForClimbSeconds = AutonomousConstants.ESTIMATED_CLIMBING_TIME_SECONDS + estimatedDriveTimeSeconds + AutonomousConstants.CLIMB_DRIVE_TIME_SAFETY_MARGIN_SECONDS;
        return MatchTracker.getMatchTimeSeconds() <= AutonomousConstants.TOTAL_MATCH_TIME_SECONDS - AutonomousConstants.AUTONOMOUS_TIME_SECONDS + timeToLeaveForClimbSeconds;
    }

    public static boolean shouldClimb() {
        return CLIMB_POSITION_CHOOSER.get() != AutonomousClimbPosition.NO_CLIMB;
    }

    private static void configureAutonomousChooser(LoggedDashboardChooser<AutonomousState> chooser) {
        chooser.addOption("Delivery", AutonomousState.DELIVERY);
        chooser.addOption("Score", AutonomousState.SCORE);
        chooser.addOption("CollectFromDepot", AutonomousState.COLLECT_FROM_DEPOT);
        chooser.addOption("CollectFromNeutralZone", AutonomousState.COLLECT_FROM_NEUTRAL_ZONE);
        chooser.addDefaultOption("Nothing", null);
    }

    private static void configureClimbPositionChooser(LoggedDashboardChooser<AutonomousClimbPosition> chooser) {
        chooser.addOption("RightL1", AutonomousClimbPosition.RIGHT_L1);
        chooser.addOption("LeftL1", AutonomousClimbPosition.LEFT_L1);
        chooser.addOption("CenterL1", AutonomousClimbPosition.CENTER_L1);
        chooser.addDefaultOption("NoClimb", AutonomousClimbPosition.NO_CLIMB);
    }

    public enum AutonomousState {
        DELIVERY(false),
        SCORE(true),
        COLLECT_FROM_DEPOT(true),
        COLLECT_FROM_NEUTRAL_ZONE(false);

        final boolean isInAllianceZone;

        AutonomousState(boolean isInAllianceZone) {
            this.isInAllianceZone = isInAllianceZone;
        }
    }

    public enum AutonomousClimbPosition {
        RIGHT_L1(FieldConstants.RIGHT_CLIMB_POSITION),
        LEFT_L1(FieldConstants.LEFT_CLIMB_POSITION),
        CENTER_L1(FieldConstants.CENTER_CLIMB_POSITION),
        NO_CLIMB(null);

        final FlippablePose2d climbPose;

        AutonomousClimbPosition(FlippablePose2d climbPose) {
            this.climbPose = climbPose;
        }
    }
}