package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Spindexer extends MotorSubsystem {
    private final TalonFXMotor motor = SpindexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private double targetVelocityRotationsPerSecond;

    public Spindexer() {
        setName("Spindexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("SpindexerMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityRotationsPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        SpindexerConstants.MECHANISM.update(
                getCurrentVelocityRotationsPerSecond(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
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
        targetVelocityRotationsPerSecond = 0;
    }

    public boolean atTargetState(SpindexerConstants.SpindexerState targetState) {
        return atVelocity(targetState.targetVelocityRotationsPerSecond);
    }

    public boolean atVelocity(double velocityRotationsPerSecond) {
        return Math.abs(getCurrentVelocityRotationsPerSecond() - velocityRotationsPerSecond)
                <= SpindexerConstants.VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND;
    }

    public Pose3d calculateComponentPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)).getRadians())
        );
        return SpindexerConstants.VISUALIZATION_ORIGIN_POSE.transformBy(yawTransform);
    }

    void setTargetState(SpindexerConstants.SpindexerState targetState) {
        setTargetVelocity(targetState.targetVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double targetVelocityRotationsPerSecond) {
        this.targetVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private double getCurrentVelocityRotationsPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }
}