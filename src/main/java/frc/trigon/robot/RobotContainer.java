// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.FuelIntakeCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetection.ObjectPoseEstimator;
import frc.trigon.robot.misc.shootingphysics.ShootingLookupTable3D;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.poseestimation.robotposeestimator.RobotPoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.hood.Hood;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.loader.Loader;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.spindexer.Spindexer;
import frc.trigon.robot.subsystems.spindexer.SpindexerCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerConstants;
import frc.trigon.robot.subsystems.turret.Turret;
import frc.trigon.robot.subsystems.turret.TurretCommands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final RobotPoseEstimator ROBOT_POSE_ESTIMATOR = new RobotPoseEstimator();
    public static final ObjectPoseEstimator OBJECT_POSE_ESTIMATOR = new ObjectPoseEstimator(
            CameraConstants.OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS,
            SimulatedGamePieceConstants.GamePieceType.FUEL,
            CameraConstants.OBJECT_DETECTION_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final Climber CLIMBER = new Climber();
    public static final Hood HOOD = new Hood();
    public static final Intake INTAKE = new Intake();
    public static final Loader LOADER = new Loader();
    public static final Shooter SHOOTER = new Shooter();
    public static final Spindexer SPINDEXER = new Spindexer();
    public static final Turret TURRET = new Turret();
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
//        configureSysIDBindings(TURRET);
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        HOOD.setDefaultCommand(HoodCommands.getRestCommand());
        INTAKE.setDefaultCommand(IntakeCommands.getDefaultCommand());
        LOADER.setDefaultCommand(LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.STOP));
        SHOOTER.setDefaultCommand(ShooterCommands.getStopCommand());
        SPINDEXER.setDefaultCommand(SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.STOP));
        TURRET.setDefaultCommand(TurretCommands.getAlignToClosestAprilTagCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());

        OperatorConstants.TOGGLE_SHOULD_KEEP_INTAKE_OPEN_TRIGGER.onTrue(FuelIntakeCommands.getToggleDefaultIntakeStateCommand());
        OperatorConstants.INTAKE_TRIGGER.whileTrue(FuelIntakeCommands.getIntakeCommand());
    }

    private void configureSysIDBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    /**
     * Initializes the general systems of the robot.
     * Some systems need to be initialized at the start of the robot code so that others can use their functions.
     * For example, the LEDConstants need to be initialized so that the other systems can use them.
     */
    private void initializeGeneralSystems() {
        Flippable.init();
        LEDConstants.init();
        AutonomousConstants.init();
        ShootingLookupTable3D.init();
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}