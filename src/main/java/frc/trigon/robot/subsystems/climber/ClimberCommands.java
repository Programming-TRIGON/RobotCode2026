package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.CLIMBER::setTargetPositionRotations,
                false,
                Set.of(RobotContainer.CLIMBER),
                "Debugging/ClimberTargetPositionRotations"
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.sysIDDrive(targetVoltage),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetExtendedStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetExtendedState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetRetractedStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                RobotContainer.CLIMBER::setTargetRetractedState,
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}