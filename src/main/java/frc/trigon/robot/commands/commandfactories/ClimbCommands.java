package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.MatchTracker;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class ClimbCommands {
    private static boolean climbCanceled = false;

    public static Command getSetDefaultCommandToRestCommand() {
        return new InstantCommand(
                () -> climbCanceled = true
        );
    }

    public static Command getClimbToL1Command() {
        return new SequentialCommandGroup(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_PREPARE).until(OperatorConstants.CONTINUE_CLIMB_TRIGGER),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_L1)
        );
    }

    public static Command getClimbDownFromL1Command() {
        return ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_DOWN);
    }

    public static Command getClimberDefaultCommand() {
        return new ConditionalCommand(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_DOWN),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST),
                () -> RobotContainer.CLIMBER.isClimbing() &&
                        MatchTracker.getMatchTimeSeconds() > 20 &&
                        MatchTracker.getMatchTimeSeconds() < 100 &&
                        !climbCanceled
        );
    }
}
