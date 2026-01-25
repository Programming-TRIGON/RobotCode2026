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
    private static boolean IS_PREPARE_TO_INTAKE = false;

    public static Command getToggleIntakeDefaultStateCommand() {
        return new InstantCommand(() ->
                IS_PREPARE_TO_INTAKE = !IS_PREPARE_TO_INTAKE
        ).andThen(
                new ConditionalCommand(
                        new InstantCommand(() ->
                                RobotContainer.INTAKE.changeDefaultCommand(
                                        IntakeCommands.getSetTargetStateCommand(
                                                IntakeConstants.IntakeState.PREPARE_TO_INTAKE
                                        )
                                )
                        ),

                        new InstantCommand(() ->
                                RobotContainer.INTAKE.changeDefaultCommand(
                                        IntakeCommands.getSetTargetStateCommand(
                                                IntakeConstants.IntakeState.REST
                                        )
                                )
                        ),

                        () -> IS_PREPARE_TO_INTAKE
                )
        );
    }

/*    public static Command getToggleIntakeDefaultStateCommand() {
        return new InstantCommand(() -> {
            IS_PREPARE_TO_INTAKE = !IS_PREPARE_TO_INTAKE;
            if (IS_PREPARE_TO_INTAKE)
                RobotContainer.INTAKE.changeDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.PREPARE_TO_INTAKE));
            else
                RobotContainer.INTAKE.changeDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST));
        });
    }*/

//    public static Command getIntakeCommand() {
//        return new ParallelCommandGroup(
//                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
//                new IntakeAssistCommand(OperatorConstants.X_ASSIST_POWER, OperatorConstants.Y_ASSIST_POWER, OperatorConstants.THETA_ASSIST_POWER).onlyIf(OperatorConstants.INTAKE_ASSIST_ENABLED)
//        );
//    }


    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                new IntakeAssistCommand(
                        OperatorConstants.X_ASSIST_POWER,
                        OperatorConstants.Y_ASSIST_POWER,
                        OperatorConstants.THETA_ASSIST_POWER
                ).onlyWhile(OperatorConstants.INTAKE_ASSIST_ENABLED).repeatedly()
        ).onlyWhile(() -> IS_PREPARE_TO_INTAKE).repeatedly();
    }
}