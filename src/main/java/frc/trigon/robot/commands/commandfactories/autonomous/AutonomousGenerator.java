package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.Flippable;
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
        configureClimbPositionChooser();
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
                getCommandFromState(0),
                getCommandFromState(1),
                getCommandFromState(2),
                getCommandFromState(3),
                getCommandFromState(4)
        );
    }

    private static Command getCommandFromState(int index) {
        AutonomousState state = getStateFromIndex(index);

        if (state == null)
            return Commands.none();

        return switch (state) {
            case DELIVERY ->
                    GeneralAutonomousCommands.getDeliveryCommand(getStateFromIndex(index - 1), () -> getDeliveryTimeout(getStateFromIndex(index + 1), getStateFromIndex(index + 2), getStateFromIndex(index + 3), getStateFromIndex(index + 4)));
            case SCORE ->
                    GeneralAutonomousCommands.getScoreCommand(getStateFromIndex(index + 1), AutonomousConstants.SCORING_TIMEOUT_SECONDS);
            case COLLECT_FROM_DEPOT ->
                    GeneralAutonomousCommands.getCollectFromDepotCommand(true, AutonomousConstants.DEPOT_COLLECTION_TIMEOUT_SECONDS);
            case COLLECT_FROM_NEUTRAL_ZONE ->
                    GeneralAutonomousCommands.getCollectFromNeutralZoneCommand(getStateFromIndex(index - 1), AutonomousConstants.NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS);
        };
    }

    private static AutonomousState getStateFromIndex(int index) {
        return switch (index) {
            case 0 -> FIRST_AUTONOMOUS_CHOOSER.get();
            case 1 -> SECOND_AUTONOMOUS_CHOOSER.get();
            case 2 -> THIRD_AUTONOMOUS_CHOOSER.get();
            case 3 -> FOURTH_AUTONOMOUS_CHOOSER.get();
            case 4 -> FIFTH_AUTONOMOUS_CHOOSER.get();
            default -> null;
        };
    }

    private static double getDeliveryTimeout(AutonomousState... nextStates) {
        double timeToLeave = AutonomousConstants.AUTONOMOUS_TIME_SECONDS;
        for (AutonomousState nextState : nextStates) {
            if (nextState == null)
                break;
            switch (nextState) {
                case SCORE -> timeToLeave -= AutonomousConstants.SCORING_TIMEOUT_SECONDS;
                case COLLECT_FROM_DEPOT -> timeToLeave -= AutonomousConstants.DEPOT_COLLECTION_TIMEOUT_SECONDS;
                case COLLECT_FROM_NEUTRAL_ZONE ->
                        timeToLeave -= AutonomousConstants.NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS;
            }
        }

        if (shouldClimb())
            timeToLeave -= calculateTimeToLeaveForClimbSeconds(getLastValidPoseOrDefault(nextStates)) + 0.5;

        return timeToLeave;
    }

    private static Translation2d getLastValidPoseOrDefault(AutonomousState[] nextStates) {
        for (int i = nextStates.length - 1; i >= 0; i--)
            if (nextStates[i] != null)
                return getExpectedEndPose(nextStates, i).get();

        return (SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_START_INTAKING_FOR_DELIVERY_POSITION : FieldConstants.LEFT_START_INTAKING_FOR_DELIVERY_POSITION).getTranslation().get();
    }

    private static Flippable<Translation2d> getExpectedEndPose(AutonomousState[] states, int validIndex) {
        return switch (states[validIndex]) {
            case DELIVERY -> SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_DELIVERY_POSITION : FieldConstants.LEFT_DELIVERY_POSITION;
            case COLLECT_FROM_NEUTRAL_ZONE -> SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_INTAKE_POSITION.getTranslation() : FieldConstants.LEFT_INTAKE_POSITION.getTranslation();
            case COLLECT_FROM_DEPOT -> FieldConstants.DEPOT_POSITION.getTranslation();
            case SCORE -> GeneralAutonomousCommands.getScoringPose(states.length > validIndex + 1 ? states[validIndex + 1] : null).getTranslation();
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
        Logger.recordOutput("Autonomous/IsInTrenchXRange", GeneralAutonomousCommands.isInTrenchXRange());
        Logger.recordOutput("Autonomous/IsInTrenchYRange", GeneralAutonomousCommands.isInTrenchYRange());
    }

    private static boolean shouldStartDrivingToClimb() {
        if (!shouldClimb() || !IS_AUTONOMOUS_CLIMB_HIGHEST_PRIORITY.get())
            return false;

        final double timeToLeaveForClimbSeconds = calculateTimeToLeaveForClimbSeconds(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
        return MatchTracker.getMatchTimeSeconds() <= timeToLeaveForClimbSeconds;
    }

    private static double calculateTimeToLeaveForClimbSeconds(Translation2d robotPose) {
        if (!shouldClimb() || !IS_AUTONOMOUS_CLIMB_HIGHEST_PRIORITY.get())
            return 0;

        final Pose2d targetClimbPose = CLIMB_POSITION_CHOOSER.get().climbPose.get();
        final double distanceToClimbPoseMeters = robotPose.getDistance(targetClimbPose.getTranslation());
        final double estimatedDriveTimeSeconds = distanceToClimbPoseMeters / AutonomousConstants.ROBOT_AVERAGE_SPEED_METERS_PER_SECOND;
        return AutonomousConstants.ESTIMATED_CLIMBING_TIME_SECONDS + estimatedDriveTimeSeconds + AutonomousConstants.CLIMB_DRIVE_TIME_SAFETY_MARGIN_SECONDS;
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

    private static void configureClimbPositionChooser() {
        AutonomousGenerator.CLIMB_POSITION_CHOOSER.addOption("RightL1", AutonomousClimbPosition.RIGHT_L1);
        AutonomousGenerator.CLIMB_POSITION_CHOOSER.addOption("LeftL1", AutonomousClimbPosition.LEFT_L1);
        AutonomousGenerator.CLIMB_POSITION_CHOOSER.addOption("CenterL1", AutonomousClimbPosition.CENTER_L1);
        AutonomousGenerator.CLIMB_POSITION_CHOOSER.addDefaultOption("NoClimb", AutonomousClimbPosition.NO_CLIMB);
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