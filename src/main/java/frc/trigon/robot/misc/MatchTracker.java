package frc.trigon.robot.misc;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

public final class MatchTracker {
    private static final double TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS = 5;

    @AutoLogOutput(key = "Assists/ShouldIndicateAllianceShift")
    public static boolean shouldIndicateAllianceShift() {
        return isHubActive(getMatchTimeSeconds() - TIME_BEFORE_ALLIANCE_SHIFT_TO_INDICATE_SECONDS);
    }

    @AutoLogOutput(key = "IsHubActive")
    public static boolean isHubActive() {
        return isHubActive(getMatchTimeSeconds());
    }

    public static boolean isHubActive(double matchTimeSeconds) {
        if (!DriverStation.isTeleop())
            return true;

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final String gameMessage = DriverStation.getGameSpecificMessage();
        if (!"R".equals(gameMessage) && !"B".equals(gameMessage))
            return true;
        final boolean didRedAllianceWinAutonomous = "R".equals(gameMessage);
        final boolean isRedHubActive = isRedHubActive(didRedAllianceWinAutonomous, matchTimeSeconds);
        return isRedAlliance == isRedHubActive;
    }

    @AutoLogOutput(key = "MatchTimeSeconds")
    public static double getMatchTimeSeconds() {
        return DriverStation.getMatchTime();
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
}