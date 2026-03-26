package frc.trigon.robot.misc.matchTracker;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.lib.utilities.Elastic;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class MatchTracker {
    private static final LoggedNetworkBoolean
            OVERRIDE_IS_HUB_ACTIVE = new LoggedNetworkBoolean("MatchTracker/OverrideIsHubActive", false),
            DID_WE_WIN_AUTONOMOUS = new LoggedNetworkBoolean("MatchTracker/DidWeWinAutonomous", false);
    private static String LAST_GAME_MESSAGE = "";

    static {
        new Trigger(MatchTracker::didGameMessageChange).onTrue(new InstantCommand(MatchTracker::setAutonomousWinner).ignoringDisable(true));
        new Trigger(
                () -> DriverStation.isTeleopEnabled() && DriverStation.isFMSAttached() && !isValidGameMessage(LAST_GAME_MESSAGE)
        ).onTrue(
                new InstantCommand(
                        () -> {
                            System.out.println("NO GAME MESSAGE! Starting Teleop without a game message! Make sure to set it manually.");
                            Elastic.sendNotification(new Elastic.Notification(Elastic.NotificationLevel.WARNING, "NO GAME MESSAGE!", "Starting Teleop without a game message! Make sure to set it manually."));
                        }
                ).ignoringDisable(true)
        );
    }

    public static boolean shouldIndicateAllianceShift() {
        return getTimeUntilAllianceShiftSeconds() <= MatchTrackerConstants.TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS;
    }

    public static boolean isHubActive() {
        final double matchTimeSeconds = getMatchTimeSeconds();

        if (isHubActive(matchTimeSeconds) || OVERRIDE_IS_HUB_ACTIVE.get())
            return true;

        if (getTimeUntilAllianceShiftSeconds(matchTimeSeconds) <= MatchTrackerConstants.MINIMUM_FUEL_DETECTION_DELAY + MatchTrackerConstants.FUEL_FLIGHT_TIME_SECONDS)
            return true;

        return getTimeSinceLastAllianceShiftSeconds(matchTimeSeconds) <= MatchTrackerConstants.HUB_DEACTIVATION_TIME_SECONDS - MatchTrackerConstants.MAXIMUM_FUEL_DETECTION_DELAY - MatchTrackerConstants.FUEL_FLIGHT_TIME_SECONDS;
    }

    public static double getTimeUntilAllianceShiftSeconds() {
        return getTimeUntilAllianceShiftSeconds(getMatchTimeSeconds());
    }

    private static boolean isHubActive(double matchTimeSeconds) {
        if (!DriverStation.isTeleop())
            return true;

        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);
        if (currentShiftNumber < 0)
            return true;
        if (DID_WE_WIN_AUTONOMOUS.get() && currentShiftNumber % 2 != 0)
            return false;
        return DID_WE_WIN_AUTONOMOUS.get() || currentShiftNumber % 2 != 0;
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
        Logger.recordOutput("MatchTracker/shiftNumber", getShiftNumber(matchTimeSeconds));
    }

    public static double getMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static double getTimeSinceLastAllianceShiftSeconds(double matchTimeSeconds) {
        return MatchTrackerConstants.ALLIANCE_SHIFT_DURATION_SECONDS - getTimeUntilAllianceShiftSeconds(matchTimeSeconds);
    }

    private static boolean didGameMessageChange() {
        final String lastGameMessage = LAST_GAME_MESSAGE;
        final String currentGameMessage = DriverStation.getGameSpecificMessage();
        LAST_GAME_MESSAGE = currentGameMessage;

        return !lastGameMessage.equals(currentGameMessage);
    }

    private static void setAutonomousWinner() {
        final String gameMessage = DriverStation.getGameSpecificMessage();
        if (!isValidGameMessage(gameMessage))
            return;

        final boolean didRedAllianceWinAutonomous = "R".equalsIgnoreCase(DriverStation.getGameSpecificMessage());
        Elastic.sendNotification(new Elastic.Notification(Elastic.NotificationLevel.INFO, "Autonomous Winner Determined", "The " + (didRedAllianceWinAutonomous ? "Red" : "Blue") + " Alliance won autonomous!"));
        DID_WE_WIN_AUTONOMOUS.set(didRedAllianceWinAutonomous && Flippable.isRedAlliance() || !didRedAllianceWinAutonomous && !Flippable.isRedAlliance());
    }

    private static boolean isValidGameMessage(String gameMessage) {
        return gameMessage != null && (gameMessage.equalsIgnoreCase("R") || gameMessage.equalsIgnoreCase("B"));
    }

    private static int getShiftNumber(double matchTimeSeconds) {
        final boolean didWeWinAutonomous = DID_WE_WIN_AUTONOMOUS.get();

        if (!didWeWinAutonomous) {
            if (matchTimeSeconds <= 130 && matchTimeSeconds >= 105) return 1;
            if (matchTimeSeconds < 105 && matchTimeSeconds > 80) return 2;
            if (matchTimeSeconds <= 80 && matchTimeSeconds >= 55) return 3;
            if (matchTimeSeconds < 55 && matchTimeSeconds > 30) return 4;
        } else {
            if (matchTimeSeconds < 130 && matchTimeSeconds > 105) return 1;
            if (matchTimeSeconds <= 105 && matchTimeSeconds >= 80) return 2;
            if (matchTimeSeconds < 80 && matchTimeSeconds > 55) return 3;
            if (matchTimeSeconds <= 55 && matchTimeSeconds >= 30) return 4;
        }

        if (matchTimeSeconds <= 30)
            return -1;
        return -2;
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
            case -1 -> 0;
            case -2 -> 130;
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