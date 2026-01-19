package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Shooter extends MotorSubsystem {
    private final TalonFXMotor motor = ShooterConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ShooterConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(ShooterConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond = 0;

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("ShooterMasterMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityMetersPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetVelocityMetersPerSecond = 0;
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ShooterConstants.SYS_ID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updateMechanism() {
        ShooterConstants.MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                targetVelocityMetersPerSecond
        );
    }

    public boolean atTargetVelocity() {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) < ShooterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    void aimAtHub() {
        final double targetVelocityFromShootingCalculations = 0/*ShootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond()*/;
        setTargetVelocity(targetVelocityFromShootingCalculations);
    }

    void aimForDelivery() {
        final double targetDeliveryVelocity = 0;//TODO: Implement
        setTargetVelocity(targetDeliveryVelocity);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond * ShooterConstants.WHEEL_SLIPPAGE_COMPENSATION_VELOCITY_MULTIPLIER;
        motor.setControl(velocityRequest.withVelocity(this.targetVelocityMetersPerSecond));
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }
}