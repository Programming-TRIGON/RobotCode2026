package frc.trigon.robot.misc.shootingphysics.shootingvisualization;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

import java.util.Random;

/**
 * A command to visualize note shooting.
 * This command will get the physical information from subsystems when we begin the shot, and calculate the note's position at each timestamp using physics.
 */
public class VisualizeFuelShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final Random RANDOM = new Random();
    private final SimulatedGamePiece shotFuel;
    private Translation3d currentFuelVelocity;
    private double currentSpinRadiansPerSecond;

    public VisualizeFuelShootingCommand(SimulatedGamePiece shotFuel) {
        this.shotFuel = shotFuel;
    }

    public static InstantCommand getScheduleShotCommand(SimulatedGamePiece shotFuel) {
        return new InstantCommand(() -> CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(shotFuel)));
    }

    @Override
    public void initialize() {
        shotFuel.updatePosition(SHOOTING_CALCULATIONS.calculateCurrentFuelExitPose());
        currentFuelVelocity = calculateFuelExitVelocityVector();
        initializeSpin(currentFuelVelocity.getNorm());
    }

    @Override
    public void execute() {
        for (int i = 0; i < (int) (RobotHardwareStats.getPeriodicTimeSeconds() / FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS); i++)
            stepSimulation();
    }

    @Override
    public boolean isFinished() {
        return shotFuel.getPosition().getZ() < FuelShootingVisualizationConstants.END_SIMULATION_HEIGHT_METERS && currentFuelVelocity.getZ() < 0;
    }

    @Override
    public void end(boolean interrupted) {
        final Translation3d currentPosition = shotFuel.getPosition();
        shotFuel.updatePosition(
                new Translation3d(
                        currentPosition.getX(),
                        currentPosition.getY(),
                        SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters
                )
        );
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
        final Translation3d dragSpeedVector = calculateCurrentDragSpeedVector(currentFuelVelocity);
        final Translation3d magnusSpeedVector = calculateCurrentMagnusSpeedVector(currentFuelVelocity);
        currentFuelVelocity = currentFuelVelocity.plus(gravitySpeedVector).plus(dragSpeedVector).plus(magnusSpeedVector);
        updateSpinDecay(currentFuelVelocity);

        final Translation3d currentGamePiecePosition = shotFuel.getPosition();
        shotFuel.updatePosition(currentGamePiecePosition.plus(currentFuelVelocity.times(FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS)));

        if (isScoredInHub())
            ejectFromHub();
    }

    private Translation3d calculateCurrentGravitySpeedVector() {
        return new Translation3d(0, 0, -FuelShootingVisualizationConstants.G_FORCE * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS);
    }

    private Translation3d calculateCurrentDragSpeedVector(Translation3d currentGamePieceVelocity) {
        final double velocityMagnitude = currentGamePieceVelocity.getNorm();
        if (velocityMagnitude < 1e-6)
            return new Translation3d();
        final double dragForceMagnitude = 0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * velocityMagnitude * velocityMagnitude * FuelShootingVisualizationConstants.DRAG_COEFFICIENT * FuelShootingVisualizationConstants.GAME_PIECE_AREA;
        final double dragAccelerationMagnitude = dragForceMagnitude / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        final double dragVelocityMagnitude = dragAccelerationMagnitude * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS;
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
        return magnusAccelerationMagnitude * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS;
    }

    private void updateSpinDecay(Translation3d currentGamePieceVelocityVector) {
        final double coefficient = (0.5 * FuelShootingVisualizationConstants.SPIN_DECAY_COEFFICIENT * FuelShootingVisualizationConstants.AIR_DENSITY * FuelShootingVisualizationConstants.GAME_PIECE_AREA) / FuelShootingVisualizationConstants.MOMENT_OF_INERTIA;
        currentSpinRadiansPerSecond -= coefficient * currentSpinRadiansPerSecond * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS * currentGamePieceVelocityVector.getNorm();
    }

    private void ejectFromHub() {
        final Translation3d ejectionPower = new Translation3d(
                getRandomNumber(SimulatedGamePieceConstants.EJECTION_FROM_HUB_MINIMUM_VELOCITY_METERS_PER_SECOND, SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_VELOCITY_METERS_PER_SECOND),
                0,
                0
        );
        final Rotation3d ejectionRotation = new Rotation3d(
                0,
                0,
                Units.degreesToRadians(
                        getRandomNumber(
                                -SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_ANGLE.getDegrees(),
                                SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_ANGLE.getDegrees()
                        )
                ) + (Flippable.isRedAlliance() ? Math.PI : 0)
        );
        final Translation3d ejectionVector = ejectionPower.rotateBy(ejectionRotation);

        shotFuel.updatePosition(SimulatedGamePieceConstants.EJECT_FUEL_FROM_HUB_POSITION.get());
        currentFuelVelocity = ejectionVector;
    }

    private boolean isScoredInHub() {
        return shotFuel.getPosition().getDistance(SimulatedGamePieceConstants.SCORE_CHECK_POSITION.get()) < SimulatedGamePieceConstants.SCORE_TOLERANCE_METERS;
    }

    private double getRandomNumber(double min, double max) {
        return min + (max - min) * RANDOM.nextDouble();
    }
}
