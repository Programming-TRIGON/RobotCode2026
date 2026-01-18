package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece>
            FUEL_ON_FIELD = SimulatedGamePieceConstants.STARTING_FUEL,
            HELD_FUEL = new ArrayList<>(List.of());
    private static double LAST_FUEL_EJECTION_TIMESTAMP = 0;

    public static ArrayList<SimulatedGamePiece> getSimulatedFuel() {
        return FUEL_ON_FIELD;
    }

    public static boolean hasFuel() {
        return !HELD_FUEL.isEmpty();
    }

    public static void update() {
        updateGamePieces();
        logGamePieces();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateFuelPeriodically();
        updateCollection();
        updateEjection();
        updateHeldFuelPoses();
    }

    /**
     * Logs the position of all the game pieces.
     */
    private static void logGamePieces() {
        final Pose3d[]
                fuelOnFieldArray = mapSimulatedGamePieceListToPoseArray(FUEL_ON_FIELD),
                heldFuelArray = mapSimulatedGamePieceListToPoseArray(HELD_FUEL),
                totalFuel = new Pose3d[fuelOnFieldArray.length + heldFuelArray.length];

        System.arraycopy(fuelOnFieldArray, 0, totalFuel, 0, fuelOnFieldArray.length);
        System.arraycopy(heldFuelArray, 0, totalFuel, fuelOnFieldArray.length, heldFuelArray.length);

        Logger.recordOutput("Poses/GamePieces/Fuel", totalFuel);
    }

    private static void updateFuelPeriodically() {
        for (SimulatedGamePiece fuel : FUEL_ON_FIELD)
            fuel.updatePeriodically();
    }

    private static void updateCollection() {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        final Pose3d robotRelativeCollectionPose = new Pose3d();//TODO: Add in intake subsystem
        final Pose3d collectionPose = robotPose.plus(toTransform(robotRelativeCollectionPose));

        updateOutpostCollection(robotPose);

        if (isCollectingFuel() && HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL) {
            final ArrayList<SimulatedGamePiece> collectedFuel = getCollectedFuel(collectionPose);
            for (SimulatedGamePiece fuel : collectedFuel) {
                if (HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL)
                    return;
                HELD_FUEL.add(fuel);
                FUEL_ON_FIELD.remove(fuel);
            }
        }
    }

    private static void updateOutpostCollection(Pose3d robotPose) {
        final double distanceFromFeeder = robotPose.toPose2d().getTranslation().getDistance(SimulatedGamePieceConstants.OUTPOST_POSITION.get());

        if (isCollectingFuelFromOutpost() && HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL &&
                distanceFromFeeder < SimulatedGamePieceConstants.OUTPOST_INTAKE_TOLERANCE_METERS) {
            HELD_FUEL.add(new SimulatedGamePiece(0, 0));
        }
    }

    /**
     * Gets the fuel object that is being collected.
     *
     * @param collectionPose the pose of the collection mechanism
     * @return the fuel object that is being collected
     */
    private static ArrayList<SimulatedGamePiece> getCollectedFuel(Pose3d collectionPose) {
        final ArrayList<SimulatedGamePiece> collectedFuel = new ArrayList<>(List.of());
        for (SimulatedGamePiece gamePiece : FUEL_ON_FIELD)
            if (gamePiece.getDistanceFromPoseMeters(collectionPose) <= SimulatedGamePieceConstants.INTAKE_TOLERANCE_METERS)
                collectedFuel.add(gamePiece);
        return collectedFuel;
    }

    private static boolean isCollectingFuel() {
        return false;//TODO: Implement
    }

    private static boolean isCollectingFuelFromOutpost() {
        return false;//TODO: Implement
    }

    private static void updateEjection() {
        if (hasFuel() && isEjectingFuel() && LAST_FUEL_EJECTION_TIMESTAMP + SimulatedGamePieceConstants.FUEL_EJECTION_DELAY_SECONDS < Timer.getFPGATimestamp()) {
            final SimulatedGamePiece heldFuel = HELD_FUEL.get(0);
            ejectGamePiece(heldFuel);
        }
    }

    private static void ejectGamePiece(SimulatedGamePiece ejectedGamePiece) {
        final Translation3d robotSelfRelativeVelocity = new Translation3d(RobotContainer.SWERVE.getSelfRelativeVelocity());
        final Translation3d robotRelativeReleaseVelocity = new Translation3d();//TODO:This should be extracted to a method in a different branch...

        ejectedGamePiece.release(robotSelfRelativeVelocity.plus(robotRelativeReleaseVelocity).rotateBy(new Rotation3d(RobotContainer.SWERVE.getHeading())));
        LAST_FUEL_EJECTION_TIMESTAMP = Timer.getFPGATimestamp();

        HELD_FUEL.remove(ejectedGamePiece);
        CommandScheduler.getInstance().schedule(new WaitUntilCommand(() -> ejectedGamePiece.getPose().getZ() <= 0).andThen(() -> FUEL_ON_FIELD.remove(ejectedGamePiece)));
    }

    private static boolean isEjectingFuel() {
        return false;//TODO: Implement
    }

    private static void updateHeldFuelPoses() {
        for (int i = 0; i < HELD_FUEL.size(); i++) {
            updateHeldFuelPose(i);
        }
    }

    private static void updateHeldFuelPose(int heldFuelIndex) {
        final SimulatedGamePiece heldFuel = HELD_FUEL.get(heldFuelIndex);
        heldFuel.updatePose(calculateHeldFuelFieldRelativePose(heldFuelIndex));
    }

    /**
     * Calculate the position of the held fuel relative to the field.
     *
     * @param heldFuelIndex the index of the fuel to calculate the position for. Used to find the spot in the magazine to use
     * @return the position of the held fuel relative to the field
     */
    private static Pose3d calculateHeldFuelFieldRelativePose(int heldFuelIndex) {
        final Pose3d robotPose3d = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        final Pose3d robotRelativeStoragePose = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_POSITIONS[heldFuelIndex];
        return robotPose3d.plus(toTransform(robotRelativeStoragePose));
    }

    /**
     * Converts a Pose3d into a Transform3d.
     *
     * @param pose the target Pose3d
     * @return the Transform3d
     */
    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    private static Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<SimulatedGamePiece> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = gamePieces.get(i).getPose();
        return poses;
    }
}