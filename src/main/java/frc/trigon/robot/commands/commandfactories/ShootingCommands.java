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
                getCalculatedAimAtHubCommand(),
                getLoadWhenReadyCommand()
        );
    }

    private static Command getLoadWhenReadyCommand() {
        return new SequentialCommandGroup(
                getWaitUntilAtTargetCommand(),
                getLoadToShooterCommand()
        );
    }

    private static Command getLoadToShooterCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.FEED_TO_TURRET),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD)
        );
    }

    private static Command getCalculatedAimAtHubCommand() {
        return new SequentialCommandGroup(
                getUpdateShootingCalculationsCommand(),
                getAimAtHubCommand()
        );
    }

    private static Command getAimAtHubCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignToHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                ShooterCommands.getAimAtHubCommand()
        );
    }

    private static Command getWaitUntilAtTargetCommand() {
        return new WaitUntilCommand(() ->
                RobotContainer.SHOOTER.atTargetVelocity()
                        && RobotContainer.HOOD.atTargetAngle()
                        && RobotContainer.TURRET.atTargetSelfRelativeAngle()
        );
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(
                ShootingCalculations.getInstance()::updateCalculations
        );
    }
}