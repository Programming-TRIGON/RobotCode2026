package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
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

    public static ArrayList<SimulatedGamePiece> getSimulatedFuel() {
        final ArrayList<SimulatedGamePiece> allSimulatedFuel = new ArrayList<>(List.of());

        allSimulatedFuel.addAll(FUEL_ON_FIELD);
        allSimulatedFuel.addAll(HELD_FUEL);

        return allSimulatedFuel;
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
        Logger.recordOutput("Poses/GamePieces/Fuel", mapSimulatedGamePieceListToPoseArray(getSimulatedFuel()));
    }

    private static void updateFuelPeriodically() {
        for (SimulatedGamePiece fuel : FUEL_ON_FIELD)
            fuel.updatePeriodically();
        for (SimulatedGamePiece fuel : HELD_FUEL)
            fuel.updatePeriodically();
    }

    private static void updateCollection() {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        final Pose3d robotRelativeCollectionPose = SimulatedGamePieceConstants.COLLECTION_CHECK_POSE;
        final Pose3d collectionPose = robotPose.plus(toTransform(robotRelativeCollectionPose));

        if (isCollectingFuel() && HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL) {
            final ArrayList<SimulatedGamePiece> collectedFuel = getCollectedFuel(collectionPose);
            for (SimulatedGamePiece fuel : collectedFuel) {
                if (HELD_FUEL.size() >= SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL)
                    return;

                HELD_FUEL.add(fuel);
                FUEL_ON_FIELD.remove(fuel);
                fuel.resetIndexing();
            }
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
        return false;//TODO: INTAKE at state
    }

    private static void updateEjection() {
        if (hasFuel() && isEjectingFuel()) {
            final SimulatedGamePiece ejectableFuel = getEjectableFuel();
            if (ejectableFuel != null)
                ejectGamePiece(ejectableFuel);
        }
    }

    private static SimulatedGamePiece getEjectableFuel() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                continue;

            final double distanceFromLoaderPose = heldFuel.getDistanceFromPoseMeters(getFuelLoaderFieldRelativePose());
            if (distanceFromLoaderPose < SimulatedGamePieceConstants.LOADER_TOLERANCE_METERS)
                return heldFuel;
        }
        return null;
    }

    private static Pose3d getFuelLoaderFieldRelativePose() {
        final Pose3d
                robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()),
                loaderPose = SimulatedGamePieceConstants.LOADER_CHECK_POSE;
        return robotPose.plus(toTransform(loaderPose));
    }

    private static void ejectGamePiece(SimulatedGamePiece ejectedGamePiece) {
        final Translation3d robotSelfRelativeVelocity = new Translation3d(RobotContainer.SWERVE.getSelfRelativeVelocity());
        final Translation3d robotRelativeReleaseVelocity = new Translation3d();//TODO: Get from SHOOTER, PITCHER, and TURRET

        ejectedGamePiece.release(robotSelfRelativeVelocity.plus(robotRelativeReleaseVelocity).rotateBy(new Rotation3d(RobotContainer.SWERVE.getHeading())));

        FUEL_ON_FIELD.add(ejectedGamePiece);
        HELD_FUEL.remove(ejectedGamePiece);

        CommandScheduler.getInstance().schedule(new WaitUntilCommand(() -> ejectedGamePiece.getPose().getZ() <= 0).andThen(() -> FUEL_ON_FIELD.remove(ejectedGamePiece)));
    }

    private static boolean isEjectingFuel() {
        return false;//TODO: SHOOTER, SPINDEXER, and LOADER at state
    }

    private static void updateHeldFuelPoses() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed()) {
                heldFuel.updatePose(calculateUnindexedHeldFuelPose());
                continue;
            }

            final Rotation2d spindexerRelativeRotation = heldFuel.getSpindexerRelativeRotation();
            heldFuel.updatePose(calculateHeldFuelFieldRelativePose(spindexerRelativeRotation));
        }
    }

    private static Pose3d calculateUnindexedHeldFuelPose() {
        return new Pose3d();//TODO: Random legitamate poses in range (?)
    }

    /**
     * Calculate the position of the held fuel relative to the field.
     *
     * @return the position of the held fuel relative to the field
     */
    private static Pose3d calculateHeldFuelFieldRelativePose(Rotation2d heldFuelSpindexerRelativeRotation) {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        final Pose3d robotRelativeSpindexerPose = RobotContainer.SPINDEXER.calculateComponentPose();

        final Rotation3d spindexerRotationOffset = new Rotation3d(heldFuelSpindexerRelativeRotation);
        final Translation3d fuelDistanceFromSpindexerOrigin = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS
                .rotateBy(spindexerRotationOffset);
        final Transform3d fuelOffsetFromSpindexerPose = new Transform3d(
                fuelDistanceFromSpindexerOrigin,
                new Rotation3d()
        );

        return robotPose.plus(toTransform(robotRelativeSpindexerPose.plus(fuelOffsetFromSpindexerPose)));
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