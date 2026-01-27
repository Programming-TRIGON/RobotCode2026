package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerConstants;
import frc.trigon.robot.subsystems.turret.TurretCommands;

public class ShootingCommands {
    public static Command getShootFuelAtHubCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(ShootingCommands::updateShootingCalculations),
                getShootAtHubCommand()
        );
    }

    public static Command getDeliverFuelCommand() {
        return new ParallelCommandGroup(
                getAimForDeliveryCommand(),
                getLoadFuelWhenReadyCommand()
        );
    }

    private static Command getShootAtHubCommand() {
        return new ParallelCommandGroup(
                new RunCommand(ShootingCommands::updateShootingCalculations),
                getAimAtHubCommand(),
                getLoadFuelWhenReadyCommand()
        );
    }

    private static Command getAimAtHubCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignToHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                ShooterCommands.getAimAtHubCommand()
        );
    }

    private static Command getAimForDeliveryCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignForDeliveryCommand(),
                HoodCommands.getAimForDeliveryCommand(),
                ShooterCommands.getAimForDeliveryCommand()
        );
    }

    private static Command getLoadFuelWhenReadyCommand() {
        return getLoadFuelCommand()
                .onlyWhile(ShootingCommands::canShoot)
                .repeatedly();
    }

    private static Command getLoadFuelCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.LOAD_TURRET),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD)
        );
    }

    private static boolean canShoot() {
        return isRobotReadyToShoot() || OperatorConstants.OVERRIDE_CAN_SHOOT_TRIGGER.getAsBoolean();
    }

    private static boolean isRobotReadyToShoot() {
        return RobotContainer.SHOOTER.atTargetVelocity()
                && RobotContainer.HOOD.atTargetAngle()
                && RobotContainer.TURRET.atTargetAngle();
    }

    private static void updateShootingCalculations() {
        ShootingCalculations.getInstance().updateCalculations();
    }
}