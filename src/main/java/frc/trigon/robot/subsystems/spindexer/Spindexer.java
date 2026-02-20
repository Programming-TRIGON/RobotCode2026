package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Spindexer extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXSMotor motor = SpindexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond;

    public Spindexer() {
        setName("Spindexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("SpindexerMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityMetersPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        SpindexerConstants.MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                motor.getSignal(TalonFXSSignal.CLOSED_LOOP_REFERENCE)
        );

//        Logger.recordOutput("Poses/Components/SpindexerPose", calculateComponentPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return SpindexerConstants.SYSID_CONFIG;
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetVelocityMetersPerSecond = 0;
    }

    public boolean atTargetState(SpindexerConstants.SpindexerState targetState) {
        return atVelocity(targetState.targetVelocityMetersPerSecond);
    }

    public boolean atVelocity(double velocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - velocityMetersPerSecond)
                <= SpindexerConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    public Pose3d calculateComponentPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, Rotation2d.fromRotations(motor.getSignal(TalonFXSSignal.POSITION)).getRadians() * SpindexerConstants.SIMULATION_SLIPPAGE_COMPENSATION_MULTIPLIER)
        );
        return SpindexerConstants.VISUALIZATION_ORIGIN_POSE.transformBy(yawTransform);
    }

    void loadToShooter() {
        final double targetShooterVelocityFromShootingCalculations = shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond();
        final double targetLoadingVelocity = targetShooterVelocityFromShootingCalculations * SpindexerConstants.LOADING_SPEED_RELATIVE_TO_SHOOTING_COEFFICIENT;
        setTargetVelocity(targetLoadingVelocity);
    }

    void setTargetState(SpindexerConstants.SpindexerState targetState) {
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSSignal.VELOCITY);
    }
}