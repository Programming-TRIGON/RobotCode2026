package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command class that limits the swerve powers to ease shooting to the hub and increase accuracy.
 */
public class ShootingSafeDriveCommand extends ParallelCommandGroup {
    private static final SlewRateLimiter
            TRANSLATION_SLEW_RATE_LIMITER = new SlewRateLimiter(0.3),
            ROTATION_SLEW_RATE_LIMITER = new SlewRateLimiter(0.8);
    private static final double
            MINIMUM_DRIVE_POWER_TOWARDS_HUB = 0.3;


    public ShootingSafeDriveCommand() {
        addCommands(
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        ShootingSafeDriveCommand::calculateSafeDriveTranslationPower,
                        ShootingSafeDriveCommand::calculateSafeDriveRotationPower
                ).asProxy()
        );
    }

    private static Translation2d calculateSafeDriveTranslationPower() {
        return limitTranslationPowerToEaseShooting(getSlewRateLimitedTranslationPower());
    }

    private static double calculateSafeDriveRotationPower() {
        final double rawPower = OperatorConstants.DRIVER_CONTROLLER.getRightX();
        return ROTATION_SLEW_RATE_LIMITER.calculate(rawPower);
    }

    private static Translation2d getSlewRateLimitedTranslationPower() {
        final Translation2d rawPower = new Translation2d(
                OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                OperatorConstants.DRIVER_CONTROLLER.getLeftX()
        );

        return new Translation2d(
                TRANSLATION_SLEW_RATE_LIMITER.calculate(rawPower.getNorm()),
                rawPower.getAngle()
        );
    }

    private static Translation2d limitTranslationPowerToEaseShooting(Translation2d targetPower) {
        final Rotation2d fieldRelativeAngleToHub = calculateFieldRelativeAngleToHub();

        final double
                rawXPower = targetPower.getX(),
                rawYPower = targetPower.getY();
        final double
                xPowerLimiter = MathUtil.clamp(Math.abs(fieldRelativeAngleToHub.getCos()), 0, 1 - MINIMUM_DRIVE_POWER_TOWARDS_HUB),
                yPowerLimiter = MathUtil.clamp(Math.abs(fieldRelativeAngleToHub.getSin()), 0, 1 - MINIMUM_DRIVE_POWER_TOWARDS_HUB);

        return new Translation2d(
                rawXPower * xPowerLimiter,
                rawYPower * yPowerLimiter
        );
    }

    private static Rotation2d calculateFieldRelativeAngleToHub() {
        final Translation2d
                robotPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation(),
                hubPosition = FieldConstants.HUB_POSITION.get();
        return hubPosition.minus(robotPosition).getAngle();
    }
}
