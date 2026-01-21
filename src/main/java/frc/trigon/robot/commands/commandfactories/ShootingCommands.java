package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.FEED_TO_TURRET),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD)
        );
    }

    public static Command getAimAtHubCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculations(),
                TurretCommands.getAlignToHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                ShooterCommands.getAimAtHubCommand()
        );
    }

    public static Command getUpdateShootingCalculations() {
        return new RunCommand(
                ShootingCalculations.getInstance()::updateCalculations
        );
    }
}
