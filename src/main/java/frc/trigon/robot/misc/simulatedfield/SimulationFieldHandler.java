package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingphysics.shootingvisualization.VisualizeFuelShootingCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece> HELD_FUEL = new ArrayList<>(List.of());

    public static boolean hasFuel() {
        return !HELD_FUEL.isEmpty();
    }

    public static void update() {
        updateGamePieces();
        SimulatedGamePiece.logAll();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateCollection();
        updateHeldFuelPoses();
        updateEjection();
    }

    private static void updateCollection() {
        final Translation3d robotRelativeCollectionPosition = SimulatedGamePieceConstants.COLLECTION_CHECK_POSITION;
        final Translation3d collectionPose = robotRelativeToFieldRelative(robotRelativeCollectionPosition);

        if (isCollectingFuel() && HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL) {
            final ArrayList<SimulatedGamePiece> collectedFuel = getCollectedFuel(collectionPose);
            for (SimulatedGamePiece fuel : collectedFuel) {
                if (HELD_FUEL.size() >= SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL)
                    return;

                HELD_FUEL.add(fuel);
                fuel.resetIndexing();
            }
        }
    }

    /**
     * Gets the fuel object that is being collected.
     *
     * @param collectionPosition the pose of the collection mechanism
     * @return the fuel object that is being collected
     */
    private static ArrayList<SimulatedGamePiece> getCollectedFuel(Translation3d collectionPosition) {
        final ArrayList<SimulatedGamePiece> collectedFuel = new ArrayList<>(List.of());
        for (SimulatedGamePiece gamePiece : SimulatedGamePiece.getUnheldGamePieces())
            if (gamePiece.getDistanceFromPositionMeters(collectionPosition) <= SimulatedGamePieceConstants.INTAKE_TOLERANCE_METERS)
                collectedFuel.add(gamePiece);
        return collectedFuel;
    }

    private static boolean isCollectingFuel() {
        return false;//TODO: INTAKE at state
    }

    private static void updateEjection() {
        if (hasFuel()) {
            final SimulatedGamePiece ejectableFuel = getEjectableFuel();
            if (ejectableFuel != null) {
                ejectGamePiece(ejectableFuel);
            }
        }
    }

    private static SimulatedGamePiece getEjectableFuel() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                continue;

            final double distanceFromLoaderPose = heldFuel.getDistanceFromPositionMeters(getFuelLoaderFieldRelativePose());
            if (distanceFromLoaderPose < SimulatedGamePieceConstants.LOADER_TOLERANCE_METERS)
                return heldFuel;
        }
        return null;
    }

    private static Translation3d getFuelLoaderFieldRelativePose() {
        final Translation3d loaderPose = SimulatedGamePieceConstants.LOADER_CHECK_POSITION;
        return robotRelativeToFieldRelative(loaderPose);
    }

    private static void ejectGamePiece(SimulatedGamePiece ejectedGamePiece) {
        ejectedGamePiece.release();
        HELD_FUEL.remove(ejectedGamePiece);
        CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(ejectedGamePiece));
    }

    private static void updateHeldFuelPoses() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed()) {
                heldFuel.resetIndexing();
                final Translation3d unindexedRobotRelativeStorePose = heldFuel.getUnindexedRobotRelativeStorePosition();
                heldFuel.updatePosition(robotRelativeToFieldRelative(unindexedRobotRelativeStorePose));
                continue;
            }

            final Rotation2d spindexerRelativeRotation = heldFuel.getSpindexerRelativeRotation();
            heldFuel.updatePosition(calculateHeldFuelFieldRelativePosition(spindexerRelativeRotation));
        }
    }

    /**
     * Calculate the position of the held fuel relative to the field.
     *
     * @return the position of the held fuel relative to the field
     */
    private static Translation3d calculateHeldFuelFieldRelativePosition(Rotation2d heldFuelSpindexerRelativeRotation) {
        final Pose3d robotRelativeSpindexerPose = RobotContainer.SPINDEXER.calculateComponentPose();

        final Rotation3d spindexerRotationOffset = new Rotation3d(heldFuelSpindexerRelativeRotation);
        final Translation3d fuelDistanceFromSpindexerOrigin = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS
                .rotateBy(spindexerRotationOffset);
        final Transform3d fuelOffsetFromSpindexerPose = new Transform3d(
                fuelDistanceFromSpindexerOrigin,
                new Rotation3d()
        );

        return robotRelativeToFieldRelative(robotRelativeSpindexerPose.plus(fuelOffsetFromSpindexerPose).getTranslation());
    }

    /**
     * Converts a robot relative pose into a field relative pose.
     *
     * @param robotRelativePose the robot relative pose to convert
     * @return the field relative pose
     */
    private static Translation3d robotRelativeToFieldRelative(Translation3d robotRelativePose) {
        final Translation3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()).getTranslation();
        return robotPose.plus(robotRelativePose);
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
}