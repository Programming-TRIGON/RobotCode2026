package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
    public static Command getToggleShouldKeepIntakeOpenCommand() {
        return new ConditionalCommand(
                new InstantCommand(
                        () -> RobotContainer.INTAKE.changeDefaultCommand(frc.trigon.robot.subsystems.intake.IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.PREPARE_TO_INTAKE))
                ),
                new InstantCommand(
                        () -> RobotContainer.INTAKE.changeDefaultCommand(frc.trigon.robot.subsystems.intake.IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST))
                ),
                OperatorConstants.SHOULD_KEEP_INTAKE_OPEN
        );
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                frc.trigon.robot.subsystems.intake.IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                new IntakeAssistCommand(
                        OperatorConstants.X_ASSIST_POWER,
                        OperatorConstants.Y_ASSIST_POWER,
                        OperatorConstants.THETA_ASSIST_POWER
                ).onlyWhile(OperatorConstants.IS_INTAKE_ASSIST_ENABLED).repeatedly()
        );
    }
}