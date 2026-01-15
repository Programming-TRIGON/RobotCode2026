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
    private final TalonFXMotor motor = SpindexerConstants.SPINDEXER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(SpindexerConstants.FOC_ENABLED);

    public Spindexer() {
        setName(SpindexerConstants.SPINDEXER_MECHANISM_NAME);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Spindexer")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/SpindexerPose", calculateComponentPose());

        SpindexerConstants.SPINDEXER_MECHANISM.update(
                getCurrentVelocity(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );
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
        Logger.recordOutput("Spindexer/CurrentVelocity", getCurrentVelocity());
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private Pose3d calculateComponentPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, getCurrentPosition())
        );
        return Pose3d.kZero.plus(yawTransform);
    }

    public Pose3d getComponentPose() {
        return calculateComponentPose();
    }

    public boolean atVelocity(double targetVelocity) {
        return Math.abs(getCurrentVelocity() - targetVelocity)
                < SpindexerConstants.VELOCITY_TOLERANCE;
    }

    public void setTargetState(SpindexerConstants.SpindexerState targetState) {
        setTargetVelocity(targetState.targetVelocity);
    }

    public void setTargetVelocity(double targetVelocity) {
        motor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    public double getCurrentVelocity() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    public double getCurrentPosition() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }
}
