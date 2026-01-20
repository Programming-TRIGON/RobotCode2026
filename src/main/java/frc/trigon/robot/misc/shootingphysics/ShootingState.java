package frc.trigon.robot.misc.shootingphysics;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingState(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                            double targetShootingVelocityMetersPerSecond) {
    public static ShootingState empty() {
        return new ShootingState(
                new Rotation2d(),
                new Rotation2d(),
                0.0
        );
    }
}
