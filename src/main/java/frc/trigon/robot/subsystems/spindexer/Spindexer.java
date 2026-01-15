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
import org.littletonrobotics.junction.Logger;

public class Spindexer extends MotorSubsystem {
    private final TalonFXMotor motor = SpindexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private double targetVelocity;

    public Spindexer() {
        setName("Spindexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Motor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        SpindexerConstants.SPINDEXER_MECHANISM.update(
                getCurrentVelocity(),
                targetVelocity
        );

        Logger.recordOutput("Poses/Components/SpindexerPose", getComponentPose());
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
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private Pose3d calculateComponentPose(Pose3d originPose) {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, getCurrentPositionRotations().getRadians())
        );
        return originPose.transformBy(yawTransform);
    }

    private Pose3d getComponentPose() {
        return calculateComponentPose(SpindexerConstants.SPINDEXER_VISUALIZATION_POSE);
    }

    boolean atVelocity() {
        return Math.abs(getCurrentVelocity() - targetVelocity)
                < SpindexerConstants.VELOCITY_TOLERANCE_ROTATIONS;
    }

    void setTargetState(SpindexerConstants.SpindexerState targetState) {
        setTargetVelocity(targetState.targetVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        motor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    double getCurrentVelocity() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    Rotation2d getCurrentPositionRotations() {
        return Rotation2d.fromRotations(getCurrentPosition());
    }

    double getCurrentPosition() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }


}