package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.Supplier;

public class IntakeCommands {
    public static Command getSetAngleMotorTargetStateCommand(IntakeConstants.AngleMotorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.AngleMotorSetTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetWheelMotorTargetStateCommand(IntakeConstants.WheelMotorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.WheelMotorSetTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.INTAKE.setTargetAngle(targetAngle.get()),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage){
        return new ExecuteEndCommand(
                () -> RobotContainer.INTAKE.setTargetVoltage(targetVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}