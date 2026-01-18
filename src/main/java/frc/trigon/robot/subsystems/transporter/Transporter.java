package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Transporter extends MotorSubsystem {
    private final TalonFXMotor motor = TransporterConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond;

    public Transporter() {
        setName("TransporterMotor");
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
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    public boolean atState(TransporterConstants.TransporterState targetState) {
        final double targetVelocity = targetState.targetVelocityMetersPerSecond;

        return atTargetVelocity(targetVelocity);
    }

    public boolean atTargetVelocity(double targetVelocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) <= TransporterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    private double getCurrentPositionMeters() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }

}
