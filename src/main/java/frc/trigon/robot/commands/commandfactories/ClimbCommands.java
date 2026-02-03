package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class ClimbCommands {
    public static Command getClimbToL1Command(Trigger continueTrigger) {
        return new SequentialCommandGroup(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_PREPARE).until(continueTrigger),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_L1)
        );
    }

    public static Command getClimbDownFromL1Command() {
        return ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_DOWN);
    }
}
