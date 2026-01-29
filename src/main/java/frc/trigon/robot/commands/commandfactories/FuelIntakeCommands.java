package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static final LoggedNetworkBoolean
            SHOULD_KEEP_INTAKE_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldKeepIntakeOpen", true),
            SHOULD_ASSIST_INTAKE = new LoggedNetworkBoolean("/SmartDashboard/ShouldAssistIntake", true);

    public static final double
            X_ASSIST_POWER = 0.0,
            Y_ASSIST_POWER = 0.5,
            THETA_ASSIST_POWER = 0.0;

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                getIntakeAssistCommand().onlyWhile(SHOULD_ASSIST_INTAKE).repeatedly()
        );
    }

    public static Command getToggleDefaultIntakeStateCommand() {
        return new InstantCommand(
                () -> SHOULD_KEEP_INTAKE_OPEN.set(!SHOULD_KEEP_INTAKE_OPEN.get())
        );
    }

    public static Command getEnableIntakeAssistCommand() {
        return new InstantCommand(
                () -> SHOULD_ASSIST_INTAKE.set(true)
        );
    }

    public static Command getDisableIntakeAssistCommand() {
        return new InstantCommand(
                () -> SHOULD_ASSIST_INTAKE.set(false)
        );
    }

    private static Command getIntakeAssistCommand() {
        return new IntakeAssistCommand(
                X_ASSIST_POWER,
                Y_ASSIST_POWER,
                THETA_ASSIST_POWER
        );
    }
}