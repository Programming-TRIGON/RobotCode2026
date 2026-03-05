package frc.trigon.robot.misc;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class MatchTracker {
    private static final LoggedNetworkBoolean
            OVERRIDE_IS_HUB_ACTIVE = new LoggedNetworkBoolean("MatchTracker/OverrideIsHubActive", false),
            DID_RED_ALLIANCE_WIN_AUTONOMOUS = new LoggedNetworkBoolean("MatchTracker/DidRedAllianceWinAutonomous", false);
    private static final double ALLIANCE_SHIFT_DURATION_SECONDS = 25;
    private static final double TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS = 5;
    private static final double HUB_DEACTIVATION_TIME_SECONDS = 3;
    private static final double MINIMUM_FUEL_DETECTION_DELAY = 1;
    private static final double MAXIMUM_FUEL_DETECTION_DELAY = 2;
    private static final double FUEL_FLIGHT_TIME_SECONDS = 1.5;
    private static String LAST_GAME_MESSAGE = "";

    static {
        new Trigger(MatchTracker::didGameMessageChange).onTrue(new InstantCommand(MatchTracker::setAutonomousWinner).ignoringDisable(true));
    }

    @AutoLogOutput(key = "Assists/ShouldIndicateAllianceShift")
    public static boolean shouldIndicateAllianceShift() {
        return getTimeUntilAllianceShiftSeconds() <= TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS;
    }

    @AutoLogOutput(key = "MatchTracker/IsHubActive")
    public static boolean isHubActive() {
        final double matchTimeSeconds = getMatchTimeSeconds();

        if (isHubActive(matchTimeSeconds) || OVERRIDE_IS_HUB_ACTIVE.get())
            return true;

        if (getTimeUntilAllianceShiftSeconds(matchTimeSeconds) <= MINIMUM_FUEL_DETECTION_DELAY + FUEL_FLIGHT_TIME_SECONDS)
            return true;

        if (getTimeSinceLastAllianceShiftSeconds(matchTimeSeconds) + MAXIMUM_FUEL_DETECTION_DELAY + FUEL_FLIGHT_TIME_SECONDS - HUB_DEACTIVATION_TIME_SECONDS <= 0)
            return true;

        return false;
    }

    public static double getTimeUntilAllianceShiftSeconds() {
        return getTimeUntilAllianceShiftSeconds(getMatchTimeSeconds());
    }

    public static boolean isHubActive(double matchTimeSeconds) {
        if (!DriverStation.isTeleop())
            return true;

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final boolean isRedHubActive = isRedHubActive(DID_RED_ALLIANCE_WIN_AUTONOMOUS.get(), matchTimeSeconds);

        return isRedAlliance == isRedHubActive;
    }

    public static double getTimeSinceLastAllianceShiftSeconds(double matchTimeSeconds) {
        return ALLIANCE_SHIFT_DURATION_SECONDS - getTimeUntilAllianceShiftSeconds(matchTimeSeconds);
    }

    public static double getTimeUntilAllianceShiftSeconds(double matchTimeSeconds) {
        return matchTimeSeconds - getNextShiftTimeSeconds(matchTimeSeconds);
    }

    public static void logMatchInfo() {
        final double matchTimeSeconds = getMatchTimeSeconds();

        Logger.recordOutput("MatchTracker/MatchTimeSeconds", matchTimeSeconds);
        Logger.recordOutput("MatchTracker/CurrentShiftType", getCurrentShiftType(matchTimeSeconds));
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilAllianceShiftSeconds", getTimeUntilAllianceShiftSeconds());
    }

    public static double getMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static boolean didGameMessageChange() {
        final String lastGameMessage = LAST_GAME_MESSAGE;
        final String currentGameMessage = DriverStation.getGameSpecificMessage();
        LAST_GAME_MESSAGE = currentGameMessage;

        return !lastGameMessage.equals(currentGameMessage);
    }

    private static void setAutonomousWinner() {
        DID_RED_ALLIANCE_WIN_AUTONOMOUS.set("R".equalsIgnoreCase(DriverStation.getGameSpecificMessage()));
    }

    private static boolean isRedHubActive(boolean didRedAllianceWinAutonomous, double matchTimeSeconds) {
        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);
        if (currentShiftNumber == -1)
            return true;
        if (didRedAllianceWinAutonomous && currentShiftNumber % 2 != 0)
            return false;
        return didRedAllianceWinAutonomous || currentShiftNumber % 2 != 0;
    }

    private static int getShiftNumber(double matchTimeSeconds) {
        if (matchTimeSeconds > 30 && matchTimeSeconds <= 55)
            return 4;
        if (matchTimeSeconds > 55 && matchTimeSeconds <= 80)
            return 3;
        if (matchTimeSeconds > 80 && matchTimeSeconds <= 105)
            return 2;
        if (matchTimeSeconds > 105 && matchTimeSeconds <= 130)
            return 1;
        return -1;
    }

    private static double getNextShiftTimeSeconds(double matchTimeSeconds) {
        if (matchTimeSeconds > 130)
            return 130;
        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);

        return switch (currentShiftNumber) {
            case 1 -> 105;
            case 2 -> 80;
            case 3 -> 55;
            case 4 -> 30;
            default -> -1;
        };
    }

    private static String getCurrentShiftType(double matchTimeSeconds) {
        if (matchTimeSeconds > 140)
            return "Autonomous";
        else if (matchTimeSeconds > 130)
            return "Transition Shift";
        else if (matchTimeSeconds < 30)
            return "Endgame";

        if (isHubActive(matchTimeSeconds))
            return "Alliance Shift";
        return "Opposing Alliance Shift";
    }
}