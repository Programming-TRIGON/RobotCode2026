package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class FuelIntakeCommand {
    private static boolean IS_INTAKE_PREPARED = false;

    public static Command getToggleIntakeDefaultStateCommand() {
        return new InstantCommand(() ->
                IS_INTAKE_PREPARED = !IS_INTAKE_PREPARED
        ).andThen(
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

                        () -> IS_INTAKE_PREPARED
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
                ).onlyWhile(OperatorConstants.INTAKE_ASSIST_ENABLED).repeatedly()
        ).onlyWhile(() -> IS_INTAKE_PREPARED).repeatedly();
    }
}