package frc.trigon.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends MotorSubsystem {
    private final TalonFXSMotor motor = SpindexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private double targetVelocityRotationsPerSecond;

    public Spindexer() {
        setName("Spindexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("SpindexerMotor")
                .angularPosition(Units.Rotations.of(getCurrentPosition().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityRotationsPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        SpindexerConstants.SPINDEXER_MECHANISM.update(
                getCurrentVelocityRotationsPerSecond(),
                motor.getSignal(TalonFXSSignal.CLOSED_LOOP_REFERENCE)
        );

        Logger.recordOutput("Poses/Components/SpindexerPose", calculateComponentPose());
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
        targetVelocityRotationsPerSecond = 0;
    }

    public boolean atTargetState(SpindexerConstants.SpindexerState targetState) {
        return atTargetVelocity(getCurrentVelocityRotationsPerSecond(), targetState.targetVelocityRotationsPerSecond);
    }

    private boolean atTargetVelocity(double currentVelocity, double targetVelocity) {
        return Math.abs(currentVelocity - targetVelocity)
                <= SpindexerConstants.VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND;
    }

    void setTargetState(SpindexerConstants.SpindexerState targetState) {
        setTargetVelocityRotationsPerSecond(targetState.targetVelocityRotationsPerSecond);
    }

    void setTargetVelocityRotationsPerSecond(double targetVelocity) {
        this.targetVelocityRotationsPerSecond = targetVelocity;
        motor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    private double getCurrentVelocityRotationsPerSecond() {
        return motor.getSignal(TalonFXSSignal.VELOCITY);
    }

    private Pose3d calculateComponentPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, getCurrentPosition().getRadians())
        );
        return SpindexerConstants.SPINDEXER_VISUALIZATION_POSE.transformBy(yawTransform);
    }

    private Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSSignal.POSITION));
    }
}