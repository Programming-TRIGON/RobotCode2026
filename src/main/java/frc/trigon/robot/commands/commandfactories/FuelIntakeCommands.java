package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static final LoggedNetworkBoolean
            SHOULD_DEFAULT_TO_OPEN_INTAKE = new LoggedNetworkBoolean("/SmartDashboard/ShouldDefaultToOpenIntake", false),
            IS_INTAKE_ASSIST_ENABLED = new LoggedNetworkBoolean("/SmartDashboard/IntakeAssistEnabled", false);

    public static Command getToggleDefaultIntakeStateCommand() {
        return new InstantCommand(
                () -> SHOULD_DEFAULT_TO_OPEN_INTAKE.set(!SHOULD_DEFAULT_TO_OPEN_INTAKE.get())
        );
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                getAssistIfEnabledCommand()
        );
    }

    private static Command getAssistIfEnabledCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                new IntakeAssistCommand(
                        OperatorConstants.X_ASSIST_POWER,
                        OperatorConstants.Y_ASSIST_POWER,
                        OperatorConstants.THETA_ASSIST_POWER
                ).asProxy(),
                Commands.idle(),
                IS_INTAKE_ASSIST_ENABLED
        );
    }
}