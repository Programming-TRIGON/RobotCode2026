package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import static frc.trigon.robot.subsystems.intake.IntakeCommands.getSetTargetStateCommand;

public class FuelIntakeCommand {
    private static boolean IS_PREPARE_TO_INTAKE = false;
    private static final double
            X_ASSIST_POWER = 0.0,
            Y_ASSIST_POWER = 0.0,
            THETA_ASSIST_POWER = 0.0;
    private static final LoggedNetworkBoolean INTAKE_ASSIST_ENABLED =
            new LoggedNetworkBoolean("Debug/intakeAssistEnabled", true);

    public static Command getToggleIntakeDefaultStateCommand() {
        return new InstantCommand(() -> {
            IS_PREPARE_TO_INTAKE = !IS_PREPARE_TO_INTAKE;
            if (IS_PREPARE_TO_INTAKE)
                RobotContainer.INTAKE.changeDefaultCommand(getSetTargetStateCommand(IntakeConstants.IntakeState.PREPARE_TO_INTAKE));
            else
                RobotContainer.INTAKE.changeDefaultCommand(getSetTargetStateCommand(IntakeConstants.IntakeState.REST));
        });
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                new IntakeAssistCommand(X_ASSIST_POWER, Y_ASSIST_POWER, THETA_ASSIST_POWER).onlyIf(INTAKE_ASSIST_ENABLED)
        );
    }
}