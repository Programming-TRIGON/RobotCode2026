package frc.trigon.robot.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.lib.hardware.misc.KeyboardController;
import frc.trigon.lib.hardware.misc.XboxController;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.misc.MatchTracker;

import java.util.function.DoubleUnaryOperator;

public class OperatorConstants {
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.07;
    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int
            DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT = 1,
            DRIVER_CONTROLLER_LEFT_STICK_EXPONENT = 2;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT, DRIVER_CONTROLLER_LEFT_STICK_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final double
            POV_DIVIDER = 2,
            TRANSLATION_STICK_SPEED_DIVIDER = 1,
            ROTATION_STICK_SPEED_DIVIDER = 1;

    public static final double MINIMUM_VELOCITY_TOWARDS_GAME_PIECE_FOR_INTAKE_ASSIST_METERS_PER_SECOND = 1;
    private static final double
            INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT = 60,
            INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_SLOPE = -15;
    public static final DoubleUnaryOperator INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA =
            x -> MathUtil.clamp(
                    (INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_SLOPE * x) + INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT,
                    0,
                    INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT
            );

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down();

    public static final Trigger
            TOGGLE_SHOULD_KEEP_INTAKE_OPEN_TRIGGER = OPERATOR_CONTROLLER.i().or(DRIVER_CONTROLLER.b()),
            INTAKE_TRIGGER = DRIVER_CONTROLLER.leftTrigger();
    public static final Trigger
            OVERRIDE_AUTO_SHOOT_TRIGGER = DRIVER_CONTROLLER.rightStick(),
            OVERRIDE_CAN_SHOOT_TRIGGER = DRIVER_CONTROLLER.leftStick(),
            SHOULD_SHOOT_TRIGGER = new Trigger(OperatorConstants::shouldShoot),
            TOGGLE_SHOULD_SHOOT_FROM_FIXED_POSITION_TRIGGER = DRIVER_CONTROLLER.back().or(OPERATOR_CONTROLLER.i()),
            RELEASE_FROM_FIXED_POSITION_TRIGGER = DRIVER_CONTROLLER.rightStick(),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_HUB_TRIGGER = OPERATOR_CONTROLLER.u(),
            SET_FIXED_SHOOTING_POSITION_LEFT_CORNER_TRIGGER = OPERATOR_CONTROLLER.h(),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_TOWER_TRIGGER = OPERATOR_CONTROLLER.j(),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_OUTPOST_TRIGGER = OPERATOR_CONTROLLER.k();

    private static boolean shouldShoot() {
        return !ShootingCommands.SHOULD_SHOOT_FROM_FIXED_POSITION.get() &&
                MatchTracker.isHubActive() &&
                isInAllianceZone() &&
                !OVERRIDE_AUTO_SHOOT_TRIGGER.getAsBoolean();
    }

    private static boolean isInAllianceZone() {
        final Pose2d currentRobotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        return currentRobotPose.getX() < FieldConstants.ALLIANCE_ZONE_LENGTH;
    }
}