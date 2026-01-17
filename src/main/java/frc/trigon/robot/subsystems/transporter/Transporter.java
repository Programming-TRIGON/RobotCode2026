package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Transporter extends MotorSubsystem {
    private final TalonFXMotor motor = TransporterConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private double targetVelocity;

    public Transporter() {
        setName("Transporter");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return TransporterConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Transporter")
                .linearPosition(Units.Meters.of(getCurrentPositionMeters()))
                .linearVelocity(Units.MetersPerSecond.of(getCurrentVelocityMetersPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        TransporterConstants.TRANSPORTER_MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );

        Logger.recordOutput("Poses/Components/TransporterPose", getComponentPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    private Pose3d getComponentPose(){
        return calculateComponentPose(TransporterConstants.TRANSPORTER_VISUALIZTION_POSE);
    }

    private Pose3d calculateComponentPose(Pose3d originPose) {
        final Transform3d transporterTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, Rotation2d.fromRotations(getCurrentPositionMeters()).getRadians(), 0)
        );
        return originPose.transformBy(transporterTransform);
    }

    public boolean atTargetState(TransporterConstants.TransporterState targetState) {
        double currentVelocity = getCurrentVelocityMetersPerSecond();
        final double targetVelocity = targetState.targetVelocityMetersPerSecond;

        return Math.abs(currentVelocity - targetVelocity) <= TransporterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    public boolean atTargetVelocity() {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocity) <= TransporterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        setTargetVelocityMetersPerSecond(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocityMetersPerSecond(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        motor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    private double getCurrentPositionMeters() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }

}
