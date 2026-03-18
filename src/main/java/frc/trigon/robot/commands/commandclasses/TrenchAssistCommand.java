package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.autonomous.SafeAutonomousDriveCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class TrenchAssistCommand extends SequentialCommandGroup {
    private static final PIDController
            LATERAL_MOVEMENT_PID_CONTROLLER =
            RobotHardwareStats.isSimulation() ?
                    new PIDController(3, 0, 0.1) :
                    new PIDController(0.9, 0, 0.01);
    private static final Rotation2d[] CAN_PASS_TRENCH_ANGLES = new Rotation2d[]{
            Rotation2d.kZero,
            Rotation2d.kPi
    };

    public TrenchAssistCommand() {
        addCommands(
                getInitializeCommand(),
                getAssistDriveThroughTrenchCommand()
        );
    }

    private static Command getInitializeCommand() {
        return new InstantCommand(LATERAL_MOVEMENT_PID_CONTROLLER::reset);
    }

    private static Command getAssistDriveThroughTrenchCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                OperatorConstants.DRIVER_CONTROLLER::getLeftY,
                TrenchAssistCommand::calculateLateralAssistPower,
                TrenchAssistCommand::calculateClosestCanPassTrenchAngle
        ).asProxy();
    }

    private static double calculateLateralAssistPower() {
        final Pose2d robotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        final double trenchYPositionMeters = SafeAutonomousDriveCommands.isRight() ?
                FieldConstants.FIELD_WIDTH_METERS - FieldConstants.TRENCH_ENTRY_Y :
                FieldConstants.TRENCH_ENTRY_Y;

        final double yOffsetFromTrench = robotPose.getY() - trenchYPositionMeters;

        return LATERAL_MOVEMENT_PID_CONTROLLER.calculate(yOffsetFromTrench);
    }

    private static FlippableRotation2d calculateClosestCanPassTrenchAngle() {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();

        Rotation2d closestCanPassTrenchAngle = CAN_PASS_TRENCH_ANGLES[0];

        for (Rotation2d canPassTrenchAngle : CAN_PASS_TRENCH_ANGLES) {
            final Rotation2d rotationOffset = robotHeading.minus(canPassTrenchAngle);
            if (Math.abs(rotationOffset.getRadians()) < Math.abs(robotHeading.minus(closestCanPassTrenchAngle).getRadians()))
                closestCanPassTrenchAngle = canPassTrenchAngle;
        }

        return new FlippableRotation2d(closestCanPassTrenchAngle, true);
    }
}