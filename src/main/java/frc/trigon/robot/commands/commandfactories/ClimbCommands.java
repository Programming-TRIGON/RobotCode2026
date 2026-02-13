package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class ClimbCommands {
    public static boolean IS_CLIMBING = false;

    public static Command getClimbToL1Command() {
        return new SequentialCommandGroup(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_PREPARE).until(OperatorConstants.CONTINUE_CLIMB_TRIGGER),
                new InstantCommand(() -> IS_CLIMBING = true),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_L1)
        ).until(OperatorConstants.CANCEL_CLIMB_TRIGGER);
    }

    public static Command getClimbDownFromL1Command() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(OperatorConstants.CANCEL_CLIMB_TRIGGER.negate()),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_DOWN).until(OperatorConstants.CANCEL_CLIMB_TRIGGER)
        );
    }

    public static Command getClimberDefaultCommand() {
        return new ConditionalCommand(
                getClimbDownFromL1Command().andThen(() -> IS_CLIMBING = false),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST),
                () -> IS_CLIMBING
        );
    }
}
