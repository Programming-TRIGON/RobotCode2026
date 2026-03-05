package frc.trigon.robot.misc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class MatchTracker {
    private static final LoggedNetworkBoolean OVERRIDE_IS_HUB_ACTIVE = new LoggedNetworkBoolean("MatchTracker/OverrideIsHubActive", false);
    private static final double TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS = 5;
    private static final double HUB_DEACTIVATION_TIME_SECONDS = 3;
    private static final double MINIMUM_FUEL_DETECTION_DELAY = 1;
    private static final double MAXIMUM_FUEL_DETECTION_DELAY = 2;
    private static final double FUEL_FLIGHT_TIME_SECONDS = 1.5;
    private static double startingMatchTime;

    public static void setStartingMatchTime() {
        startingMatchTime = Timer.getFPGATimestamp();
    }

    @AutoLogOutput(key = "MatchTracker/CurrentShift")
    public static String getCurrentShiftName() {
        if (getMatchTimeSeconds() <= 20)
            return "Autonomous";
        if (getMatchTimeSeconds() <= 30)
            return "Transition Shift";
        if (getMatchTimeSeconds() <= 55)
            return "Shift 1";
        return "";
    }

    @AutoLogOutput(key = "Assists/ShouldIndicateAllianceShift")
    public static boolean shouldIndicateAllianceShift() {
        return DriverStation.getMatchType() != DriverStation.MatchType.None &&
                isHubActive(getMatchTimeSeconds() - TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS);
    }

    @AutoLogOutput(key = "MatchTracker/IsHubActive")
    public static boolean isHubActive() {
        return isHubActive(getMatchTimeSeconds()) || OVERRIDE_IS_HUB_ACTIVE.get();
    }

    public static boolean isHubActive(double matchTimeSeconds) {
        if (!DriverStation.isTeleop())
            return true;

        if (!isHubActive(matchTimeSeconds)) {
            final int currentShift = getShiftNumber(matchTimeSeconds);
            if (getTimeUntilShiftEnd(currentShift) <= MINIMUM_FUEL_DETECTION_DELAY + FUEL_FLIGHT_TIME_SECONDS)
                return true;
            if (getTimeSinceShiftStart(currentShift) + MAXIMUM_FUEL_DETECTION_DELAY + FUEL_FLIGHT_TIME_SECONDS - HUB_DEACTIVATION_TIME_SECONDS <= 0)
                return true;
        }

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final String gameMessage = DriverStation.getGameSpecificMessage();
        if (!"R".equals(gameMessage) && !"B".equals(gameMessage))
            return true;
        final boolean didRedAllianceWinAutonomous = "R".equals(gameMessage);
        final boolean isRedHubActive = isRedHubActive(didRedAllianceWinAutonomous, matchTimeSeconds);
        return isRedAlliance == isRedHubActive;
    }


    public static double getMatchTimeSeconds() {
        return Timer.getFPGATimestamp() - startingMatchTime;
    }

    private static boolean isRedHubActive(boolean didRedAllianceWinAutonomous, double matchTimeSeconds) {
        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);
        if (currentShiftNumber == -1)
            return true;
        if (didRedAllianceWinAutonomous && currentShiftNumber % 2 != 0)
            return false;
        if (!didRedAllianceWinAutonomous && currentShiftNumber % 2 == 0)
            return false;
        return true;
    }

    private static double getTimeUntilShiftEnd(int shift) {
        return getShiftEndTimeStamp(shift) - getMatchTimeSeconds();
    }

    private static double getTimeSinceShiftStart(int shift) {
        return getMatchTimeSeconds() - getShiftStartTimeStamp(shift);
    }

    private static double getShiftEndTimeStamp(int shift) {
        return getShiftStartTimeStamp(shift) + 25;
    }

    private static double getShiftStartTimeStamp(int shift) {
        return switch (shift) {
            case 1 -> 30;
            case 2 -> 55;
            case 3 -> 80;
            case 4 -> 105;
            default -> 0;
        };
    }

    private static int getShiftNumber(double matchTimeSeconds) {
        if (matchTimeSeconds > 30 && matchTimeSeconds <= 55)
            return 1;
        if (matchTimeSeconds > 55 && matchTimeSeconds <= 80)
            return 2;
        if (matchTimeSeconds > 80 && matchTimeSeconds <= 105)
            return 3;
        if (matchTimeSeconds > 105 && matchTimeSeconds <= 130)
            return 4;
        return -1;
    }
}