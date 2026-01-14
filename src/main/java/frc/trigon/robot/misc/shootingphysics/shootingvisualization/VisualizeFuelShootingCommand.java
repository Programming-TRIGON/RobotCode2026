package frc.trigon.robot.misc.shootingphysics.shootingvisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;

/**
 * A command to visualize note shooting.
 * This command will get the physical information from subsystems when we begin the shot, and calculate the note's position at each timestamp using physics.
 */
public class VisualizeFuelShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private Translation3d currentGamePiecePosition, currentGamePieceVelocity;
    private double currentSpinRadiansPerSecond;

    @Override
    public void initialize() {
        currentGamePiecePosition = SHOOTING_CALCULATIONS.calculateCurrentFuelExitPose();
        currentGamePieceVelocity = calculateFuelExitVelocityVector();
        initializeSpin(currentGamePieceVelocity.getNorm());
    }

    @Override
    public void execute() {
        final Translation3d gravitySpeedVector = calculateCurrentGravitySpeedVector();
        final Translation3d dragSpeedVector = calculateCurrentDragSpeedVector(currentGamePieceVelocity);
        final Translation3d magnusSpeedVector = calculateCurrentMagnusSpeedVector(currentGamePieceVelocity);
        updateSpin(gravitySpeedVector);
        currentGamePieceVelocity = currentGamePieceVelocity.plus(gravitySpeedVector).plus(dragSpeedVector).plus(magnusSpeedVector);
        currentGamePiecePosition = currentGamePiecePosition.plus(currentGamePieceVelocity.times(FuelShootingVisualizationConstants.TIME_STEP_SECONDS));
    }

    @Override
    public boolean isFinished() {
        return currentGamePiecePosition.getZ() < 0.6 && currentGamePieceVelocity.getZ() < 0;
    }

    @Override
    public void end(boolean interrupted) {
    }

    private Translation3d calculateFuelExitVelocityVector() {
        final Translation3d shootingVelocityVector = calculateShootingVelocityVector();
        final Translation3d robotVelocityVector = new Translation3d(RobotContainer.SWERVE.getFieldRelativeVelocity()); // TODO: get from

        return shootingVelocityVector.minus(robotVelocityVector); // TODO: check direction
    }

    private Translation3d calculateShootingVelocityVector() {
        final double fuelExitSpeedMetersPerSecond = 0; // TODO: get from shooter subsystem
        final Rotation2d fuelExitAngle = new Rotation2d(); // TODO: get from hood subsystem
        final Rotation2d turretFieldRelativeAngle = new Rotation2d(); // TODO: get from turret subsystem
        return new Translation3d(fuelExitSpeedMetersPerSecond, new Rotation3d(0, -fuelExitAngle.getRadians(), turretFieldRelativeAngle.getRadians()));
    }

    private void initializeSpin(double fuelExitVelocityMetersPerSecond) {
        final double spinConstant = (FuelShootingVisualizationConstants.BOTTOM_TRACTION_COEFFICIENT - FuelShootingVisualizationConstants.TOP_TRACTION_COEFFICIENT) / (FuelShootingVisualizationConstants.BOTTOM_TRACTION_COEFFICIENT + FuelShootingVisualizationConstants.TOP_TRACTION_COEFFICIENT);
        currentSpinRadiansPerSecond = (spinConstant * fuelExitVelocityMetersPerSecond) / (FuelShootingVisualizationConstants.GAME_PIECE_RADIUS_METERS * 2 * Math.PI);
    }

    private Translation3d calculateCurrentGravitySpeedVector() {
        return new Translation3d(0, 0, -FuelShootingVisualizationConstants.G_FORCE * FuelShootingVisualizationConstants.TIME_STEP_SECONDS);
    }

    private Translation3d calculateCurrentDragSpeedVector(Translation3d currentGamePieceVelocity) {
        final double velocityMagnitude = currentGamePieceVelocity.getNorm();
        final double dragForceMagnitude = 0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * velocityMagnitude * velocityMagnitude * FuelShootingVisualizationConstants.DRAG_COEFFICIENT * FuelShootingVisualizationConstants.GAME_PIECE_AREA;
        final double dragAccelerationMagnitude = dragForceMagnitude / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        final double dragVelocityMagnitude = dragAccelerationMagnitude * FuelShootingVisualizationConstants.TIME_STEP_SECONDS;
        return new Translation3d(-dragVelocityMagnitude, getAngle(currentGamePieceVelocity));
    }

    private Translation3d calculateCurrentMagnusSpeedVector(Translation3d currentGamePieceVelocity) {
        final double gamePieceVelocityMagnitude = currentGamePieceVelocity.getNorm();
        final double spinParameter = (currentSpinRadiansPerSecond * FuelShootingVisualizationConstants.GAME_PIECE_RADIUS_METERS) / (gamePieceVelocityMagnitude);
        final double magnusLiftCoefficient = FuelShootingVisualizationConstants.MAGNUS_LIFT_FACTOR * spinParameter;
        final double magnusAccelerationMagnitude = (0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * gamePieceVelocityMagnitude * gamePieceVelocityMagnitude * magnusLiftCoefficient * FuelShootingVisualizationConstants.GAME_PIECE_AREA) / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        return new Translation3d(magnusAccelerationMagnitude * FuelShootingVisualizationConstants.TIME_STEP_SECONDS, getAngle(currentGamePieceVelocity).plus(new Rotation3d(Rotation2d.kCCW_90deg)));
    }

    private void updateSpin(Translation3d currentGamePieceVelocity) {
        final double coefficient = (0.5 * FuelShootingVisualizationConstants.SPIN_DECAY_COEFFICIENT * FuelShootingVisualizationConstants.AIR_DENSITY * FuelShootingVisualizationConstants.GAME_PIECE_AREA) / FuelShootingVisualizationConstants.MOMENT_OF_INERTIA;
        currentSpinRadiansPerSecond -= coefficient * currentSpinRadiansPerSecond * FuelShootingVisualizationConstants.TIME_STEP_SECONDS * currentGamePieceVelocity.getNorm();
    }

    private Rotation3d getAngle(Translation3d translation) {
        return new Rotation3d(0, -SHOOTING_CALCULATIONS.getYaw(translation).getRadians(), SHOOTING_CALCULATIONS.getPitch(translation).getRadians()); // TODO: check signs
    }
}
