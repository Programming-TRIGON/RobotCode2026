package frc.trigon.robot.misc;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

public class MatchTracker {
    @AutoLogOutput(key = "IsHubActive")
    public static boolean isHubActive() {
        if (!DriverStation.isTeleop())
            return true;

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final boolean didRedAllianceWinAutonomous = DriverStation.getGameSpecificMessage().equals("R");
        final boolean isRedHubActive = isRedHubActive(didRedAllianceWinAutonomous);
        return isRedAlliance ? isRedHubActive : !isRedHubActive;
    }

    @AutoLogOutput(key = "MatchTimeSeconds")
    public static double getMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static boolean isRedHubActive(boolean didRedAllianceWinAutonomous) {
        final int currentShiftNumber = getCurrentShiftNumber();
        if (currentShiftNumber == -1)
            return true;
        if (didRedAllianceWinAutonomous && (currentShiftNumber == 1 || currentShiftNumber == 3))
            return false;
        if (!didRedAllianceWinAutonomous && (currentShiftNumber == 2 || currentShiftNumber == 4))
            return false;
        return true;
    }

    private static int getCurrentShiftNumber() {
        final double matchTimeSeconds = getMatchTimeSeconds();
        if (matchTimeSeconds >= 30 && matchTimeSeconds <= 55)
            return 4;
        if (matchTimeSeconds >= 55 && matchTimeSeconds <= 80)
            return 3;
        if (matchTimeSeconds >= 80 && matchTimeSeconds <= 105)
            return 2;
        if (matchTimeSeconds >= 105 && matchTimeSeconds <= 130)
            return 1;
        return -1;
    }
}