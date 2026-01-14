package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;

public record ShootingState(FlippableRotation2d targetRobotAngle, Rotation2d targetPitch,
                            double targetShootingVelocityMetersPerSecond) {
    public static ShootingState empty() {
        return new ShootingState(
                new FlippableRotation2d(new Rotation2d(), false),
                new Rotation2d(),
                0.0
        );
    }
}
