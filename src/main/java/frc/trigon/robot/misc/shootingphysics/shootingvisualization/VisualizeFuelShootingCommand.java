package frc.trigon.robot.misc.shootingphysics.shootingvisualization;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A command to visualize note shooting.
 * This command will get the physical information from subsystems when we begin the shot, and calculate the note's position at each timestamp using physics.
 */
public class VisualizeFuelShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final ArrayList<Translation3d> VISUALIZED_POSITIONS = new ArrayList<>();
    private Translation3d currentGamePiecePosition, currentGamePieceVelocity;
    private double currentSpinRadiansPerSecond;

    private VisualizeFuelShootingCommand() {
    }

    public static InstantCommand getScheduleShotCommand() {
        return new InstantCommand(() -> CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand()));
    }

    @Override
    public void initialize() {
        currentGamePiecePosition = SHOOTING_CALCULATIONS.calculateCurrentFuelExitPose();
        VISUALIZED_POSITIONS.add(currentGamePiecePosition);
        currentGamePieceVelocity = calculateFuelExitVelocityVector();
        initializeSpin(currentGamePieceVelocity.getNorm());
    }

    @Override
    public void execute() {
        for (int i = 0; i < (int) (0.02 / FuelShootingVisualizationConstants.TIME_STEP_SECONDS); i++)
            stepSimulation();

        Logger.recordOutput("Poses/GamePieces/ShotFuelPoses", VISUALIZED_POSITIONS.toArray(Translation3d[]::new));
    }

    @Override
    public boolean isFinished() {
        return currentGamePiecePosition.getZ() < 0.6 && currentGamePieceVelocity.getZ() < 0;
    }

    @Override
    public void end(boolean interrupted) {
        VISUALIZED_POSITIONS.remove(currentGamePiecePosition);
    }

    private Translation3d calculateFuelExitVelocityVector() {
        final Translation3d shootingVelocityVector = calculateShootingVelocityVector();
        final Translation3d robotVelocityVector = new Translation3d(RobotContainer.SWERVE.getFieldRelativeVelocity());

        return shootingVelocityVector.plus(robotVelocityVector);
    }

    private Translation3d calculateShootingVelocityVector() {
        final double fuelExitSpeedMetersPerSecond = RobotContainer.SHOOTER.getCurrentVelocityMetersPerSecond();
        final Rotation2d fuelExitPitch = RobotContainer.HOOD.getCurrentAngle();
        final Rotation2d turretFieldRelativeAngle = RobotContainer.TURRET.getCurrentFieldRelativeAngle();
        return new Translation3d(fuelExitSpeedMetersPerSecond, new Rotation3d(0, -fuelExitPitch.getRadians(), turretFieldRelativeAngle.getRadians()));
    }

    private void initializeSpin(double fuelExitVelocityMetersPerSecond) {
        final double spinConstant = (FuelShootingVisualizationConstants.BOTTOM_TRACTION_COEFFICIENT - FuelShootingVisualizationConstants.TOP_TRACTION_COEFFICIENT) / (FuelShootingVisualizationConstants.BOTTOM_TRACTION_COEFFICIENT + FuelShootingVisualizationConstants.TOP_TRACTION_COEFFICIENT);
        currentSpinRadiansPerSecond = (2 * spinConstant * fuelExitVelocityMetersPerSecond) / (FuelShootingVisualizationConstants.GAME_PIECE_RADIUS_METERS);
    }

    private void stepSimulation() {
        final Translation3d gravitySpeedVector = calculateCurrentGravitySpeedVector();
        final Translation3d dragSpeedVector = calculateCurrentDragSpeedVector(currentGamePieceVelocity);
        final Translation3d magnusSpeedVector = calculateCurrentMagnusSpeedVector(currentGamePieceVelocity);
        currentGamePieceVelocity = currentGamePieceVelocity.plus(gravitySpeedVector).plus(dragSpeedVector).plus(magnusSpeedVector);
        updateSpinDecay(currentGamePieceVelocity);

        VISUALIZED_POSITIONS.remove(currentGamePiecePosition);
        currentGamePiecePosition = currentGamePiecePosition.plus(currentGamePieceVelocity.times(FuelShootingVisualizationConstants.TIME_STEP_SECONDS));
        VISUALIZED_POSITIONS.add(currentGamePiecePosition);
    }

    private Translation3d calculateCurrentGravitySpeedVector() {
        return new Translation3d(0, 0, -FuelShootingVisualizationConstants.G_FORCE * FuelShootingVisualizationConstants.TIME_STEP_SECONDS);
    }

    private Translation3d calculateCurrentDragSpeedVector(Translation3d currentGamePieceVelocity) {
        final double velocityMagnitude = currentGamePieceVelocity.getNorm();
        if (velocityMagnitude < 1e-6)
            return new Translation3d();
        final double dragForceMagnitude = 0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * velocityMagnitude * velocityMagnitude * FuelShootingVisualizationConstants.DRAG_COEFFICIENT * FuelShootingVisualizationConstants.GAME_PIECE_AREA;
        final double dragAccelerationMagnitude = dragForceMagnitude / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        final double dragVelocityMagnitude = dragAccelerationMagnitude * FuelShootingVisualizationConstants.TIME_STEP_SECONDS;
        final Translation3d velocityDirection = currentGamePieceVelocity.div(velocityMagnitude);

        return velocityDirection.times(-dragVelocityMagnitude);
    }

    private Translation3d calculateCurrentMagnusSpeedVector(Translation3d currentGamePieceVelocity) {
        final double gamePieceVelocityMagnitude = currentGamePieceVelocity.getNorm();
        if (gamePieceVelocityMagnitude < 1e-6)
            return new Translation3d();

        final double magnusVelocityMagnitude = calculateMagnusVelocityMagnitude(gamePieceVelocityMagnitude);

        final Vector<N3> magnusDirection = FuelShootingVisualizationConstants.MAGNUS_SPIN_AXIS.cross(currentGamePieceVelocity);
        final double magnusDirectionNorm = magnusDirection.norm();
        if (magnusDirectionNorm < 1e-6) return new Translation3d();

        final Vector<N3> magnusUnit = magnusDirection.div(magnusDirectionNorm);
        final Vector<N3> magnusVelocityVector = magnusUnit.times(magnusVelocityMagnitude);
        return new Translation3d(
                magnusVelocityVector.get(0),
                magnusVelocityVector.get(1),
                magnusVelocityVector.get(2)
        );
    }

    private double calculateMagnusVelocityMagnitude(double gamePieceVelocityMagnitude) {
        final double spinParameter = (currentSpinRadiansPerSecond * FuelShootingVisualizationConstants.GAME_PIECE_RADIUS_METERS) / gamePieceVelocityMagnitude;
        final double magnusLiftCoefficient = FuelShootingVisualizationConstants.MAGNUS_LIFT_FACTOR * spinParameter;
        final double magnusAccelerationMagnitude = (0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * gamePieceVelocityMagnitude * gamePieceVelocityMagnitude * magnusLiftCoefficient * FuelShootingVisualizationConstants.GAME_PIECE_AREA) / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        return magnusAccelerationMagnitude * FuelShootingVisualizationConstants.TIME_STEP_SECONDS;
    }

    private void updateSpinDecay(Translation3d currentGamePieceVelocityVector) {
        final double coefficient = (0.5 * FuelShootingVisualizationConstants.SPIN_DECAY_COEFFICIENT * FuelShootingVisualizationConstants.AIR_DENSITY * FuelShootingVisualizationConstants.GAME_PIECE_AREA) / FuelShootingVisualizationConstants.MOMENT_OF_INERTIA;
        currentSpinRadiansPerSecond -= coefficient * currentSpinRadiansPerSecond * FuelShootingVisualizationConstants.TIME_STEP_SECONDS * currentGamePieceVelocityVector.getNorm();
    }
}
