package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerConstants;
import frc.trigon.robot.subsystems.turret.TurretCommands;

public class ShootingCommands {
    public static Command getShootFuelCommand() {
        return new ParallelCommandGroup(
                getAimAtHubCommand(),
                getFeedToShooterCommand()
        );
    }

    private static Command getFeedToShooterCommand() {
        return new RunCommand(() -> {
            if (RobotContainer.TURRET.atTargetSelfRelativeAngle()
                    && RobotContainer.HOOD.atTargetAngle()
                    && RobotContainer.SHOOTER.atTargetVelocity()) {
                CommandScheduler.getInstance().schedule(
                        LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD),
                        SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.FEED_TO_TURRET)
                );
            }
        });
    }


    private static Command getAimAtHubCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculations(),
                TurretCommands.getAlignToHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                ShooterCommands.getAimAtHubCommand()
        );
    }

    private static Command getUpdateShootingCalculations() {
        return new RunCommand(
                ShootingCalculations.getInstance()::updateCalculations
        );
    }
}
