package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.robot.RobotContainer;

import java.util.function.Supplier;

public class IntakeCommands {
    public static Command getSetAngleMotorTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetWheelMotorTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetAngle(targetAngle.get()),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage){
        return new ExecuteEndCommand(
                () -> RobotContainer.INTAKE.setTargetIntakeVoltage(targetVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}