package frc.trigon.robot.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.lib.hardware.misc.KeyboardController;
import frc.trigon.lib.hardware.misc.XboxController;

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

    public static final Trigger//General Triggers
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down();

    public static final Trigger//Intake Triggers
            INTAKE_TRIGGER = DRIVER_CONTROLLER.leftTrigger(),
            TOGGLE_SHOULD_KEEP_INTAKE_OPEN_TRIGGER = DRIVER_CONTROLLER.b().or(OPERATOR_CONTROLLER.u()),
            ENABLE_INTAKE_ASSIST = OPERATOR_CONTROLLER.o(),
            DISABLE_INTAKE_ASSIST = DRIVER_CONTROLLER.a().or(OPERATOR_CONTROLLER.p());
    public static final Trigger//Shooting Triggers
            OVERRIDE_AUTO_SHOOT_TRIGGER = DRIVER_CONTROLLER.rightStick(),
            SHOULD_SHOOT_TRIGGER = OVERRIDE_AUTO_SHOOT_TRIGGER.negate(),
            SHOOT_FROM_FIXED_POSITION_TRIGGER = DRIVER_CONTROLLER.rightBumper().and(OVERRIDE_AUTO_SHOOT_TRIGGER),
            FIXED_DELIVERY_TRIGGER = DRIVER_CONTROLLER.leftBumper().and(OVERRIDE_AUTO_SHOOT_TRIGGER),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_HUB_TRIGGER = DRIVER_CONTROLLER.povUp().and(OPERATOR_CONTROLLER.i()),
            SET_FIXED_SHOOTING_POSITION_LEFT_CORNER_TRIGGER = DRIVER_CONTROLLER.povLeft().and(OPERATOR_CONTROLLER.j()),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_TOWER_TRIGGER = DRIVER_CONTROLLER.povDown().and(OPERATOR_CONTROLLER.k()),
            SET_FIXED_SHOOTING_POSITION_CLOSE_TO_OUTPOST_TRIGGER = DRIVER_CONTROLLER.povRight().and(OPERATOR_CONTROLLER.l());
    public static final Trigger//Climb Triggers
            OPEN_CLIMBER_TRIGGER = DRIVER_CONTROLLER.back();
    //          CANCEL_CLIMB_TRIGGER = new Trigger(ClimberCommands::IS_CLIMBING).and(DRIVER_CONTROLLER.leftBumper()),
    //          CONTINUE_CLIMB_TRIGGER = new Trigger(ClimberCommands::IS_CLIMBING).and(DRIVER_CONTROLLER.rightBumper());
    public static final Trigger//Debugging Triggers
            UNJAM_TRIGGER = DRIVER_CONTROLLER.start(),
            SHORT_EJECTION_TRIGGER = DRIVER_CONTROLLER.x();
}