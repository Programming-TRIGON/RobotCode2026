package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A command class that limits the swerve powers to ease shooting to the hub and increase accuracy.
 */
public class ShootingSafeDriveCommand extends SequentialCommandGroup {
    private static final double
            TRANSLATION_SLEW_RATE = 1.5,
            MAXIMUM_DRIVE_POWER_TOWARDS_HUB = 2.5 / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;
    private static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(TRANSLATION_SLEW_RATE),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(TRANSLATION_SLEW_RATE),
            ROTATION_SLEW_RATE_LIMITER = new SlewRateLimiter(0.8);

    public ShootingSafeDriveCommand() {
        addCommands(
                getInitializeCommand(),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        ShootingSafeDriveCommand::calculateSafeDriveTranslationPower,
                        ShootingSafeDriveCommand::calculateSafeDriveRotationPower
                ).asProxy()
        );
    }

    private static Command getInitializeCommand() {
        return new InstantCommand(() -> {
            X_SLEW_RATE_LIMITER.reset(0);
            Y_SLEW_RATE_LIMITER.reset(0);
            ROTATION_SLEW_RATE_LIMITER.reset(0);
        });
    }

    private static Translation2d calculateSafeDriveTranslationPower() {
        return limitTranslationPowerToEaseShooting(getSlewRateLimitedTranslationPower());
    }

    private static double calculateSafeDriveRotationPower() {
        final double rawPower = OperatorConstants.DRIVER_CONTROLLER.getRightX();
        return ROTATION_SLEW_RATE_LIMITER.calculate(rawPower);
    }

    private static Translation2d getSlewRateLimitedTranslationPower() {
        final double
                rawXPower = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                rawYPower = OperatorConstants.DRIVER_CONTROLLER.getLeftX();

        return new Translation2d(
                X_SLEW_RATE_LIMITER.calculate(rawXPower),
                Y_SLEW_RATE_LIMITER.calculate(rawYPower)
        );
    }

    private static Translation2d limitTranslationPowerToEaseShooting(Translation2d targetPower) {
        final Rotation2d hubDirection = calculateFieldRelativeAngleToHub().plus(Rotation2d.k180deg);

        final double radialVelocity = calculateRadialVelocityToHub(targetPower, hubDirection);
        final double limitedRadialVelocity = limitRadialVelocity(radialVelocity);

        final Translation2d tangentialComponent = calculateTangentialComponent(targetPower, radialVelocity, hubDirection);
        final Translation2d limitedRadialComponent = calculateRadialComponent(limitedRadialVelocity, hubDirection);

        return tangentialComponent.plus(limitedRadialComponent);
    }

    private static double calculateRadialVelocityToHub(Translation2d velocity, Rotation2d hubDirection) {
        return velocity.getX() * hubDirection.getCos() + velocity.getY() * hubDirection.getSin();
    }

    private static double limitRadialVelocity(double radialVelocity) {
        return MathUtil.clamp(
                radialVelocity,
                -MAXIMUM_DRIVE_POWER_TOWARDS_HUB,
                MAXIMUM_DRIVE_POWER_TOWARDS_HUB
        );
    }

    private static Translation2d calculateTangentialComponent(Translation2d velocity, double radialVelocity, Rotation2d hubDirection) {
        final Translation2d radialComponent = calculateRadialComponent(radialVelocity, hubDirection);
        return velocity.minus(radialComponent);
    }

    private static Translation2d calculateRadialComponent(double radialVelocity, Rotation2d hubDirection) {
        return new Translation2d(
                radialVelocity * hubDirection.getCos(),
                radialVelocity * hubDirection.getSin()
        );
    }

    private static Rotation2d calculateFieldRelativeAngleToHub() {
        final Translation2d
                robotPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation(),
                hubPosition = FieldConstants.HUB_POSITION.get();
        return hubPosition.minus(robotPosition).getAngle();
    }
}
