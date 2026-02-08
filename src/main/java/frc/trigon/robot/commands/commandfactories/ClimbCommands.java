package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ClimbCommands {
    public static LoggedNetworkBoolean IS_CLIMBING = new LoggedNetworkBoolean("IsClimbing", false);

    public static Command getClimbToL1Command() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_CLIMBING.set(true)),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_CLIMB).until(OperatorConstants.CONTINUE_CLIMB_TRIGGER),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_L1)
        ).until(OperatorConstants.CANCEL_CLIMB_TRIGGER);
    }

    public static Command getReleaseL1Command() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(OperatorConstants.CANCEL_CLIMB_TRIGGER.negate()),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.RELEASE_CLIMB).until(OperatorConstants.CANCEL_CLIMB_TRIGGER)
        );
    }

    public static Command getClimberDefaultCommand() {
        return new ConditionalCommand(
                getReleaseL1Command().andThen(() -> IS_CLIMBING.set(false)),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST),
                IS_CLIMBING::get
        );
    }
}
