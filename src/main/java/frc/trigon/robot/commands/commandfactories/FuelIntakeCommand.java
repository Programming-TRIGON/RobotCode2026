package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class FuelIntakeCommand {
/*    public static Command getToggleIntakePreparedCommand() {
        return new InstantCommand(() -> OperatorConstants.SHOULD_KEEP_INTAKE_OPEN = !OperatorConstants.SHOULD_KEEP_INTAKE_OPEN);
    }

    *//** Default command: continuously applies PREPARE_TO_INTAKE or REST based on the flag *//*
    public static Command getIntakePreparedDefaultCommand() {
        return Commands.either(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.PREPARE_TO_INTAKE),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST),
                () -> OperatorConstants.SHOULD_KEEP_INTAKE_OPEN
        ).repeatedly();
    }*/

    public static Command getToggleShouldKeepIntakeOpenCommand() {
        return new InstantCommand(() -> {
        }).andThen(
                new ConditionalCommand(
                        new InstantCommand(() ->
                                RobotContainer.INTAKE.changeDefaultCommand(
                                        IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.PREPARE_TO_INTAKE)
                                )
                        ),
                        new InstantCommand(() ->
                                RobotContainer.INTAKE.changeDefaultCommand(
                                        IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST)
                                )
                        ),
                        OperatorConstants.SHOULD_KEEP_INTAKE_OPEN::getAsBoolean
                )
        );
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                new IntakeAssistCommand(
                        OperatorConstants.X_ASSIST_POWER,
                        OperatorConstants.Y_ASSIST_POWER,
                        OperatorConstants.THETA_ASSIST_POWER
                ).onlyWhile(OperatorConstants.IS_INTAKE_ASSIST_ENABLED).repeatedly()
        );
    }
}