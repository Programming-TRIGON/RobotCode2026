package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Loader extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = LoaderConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(LoaderConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(LoaderConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond;

    public Loader() {
        setName("Loader");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return LoaderConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("LoaderMotor")
                .linearPosition(Units.Meters.of(motor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(getCurrentVelocityMetersPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        LoaderConstants.LOADER_MECHANISM.update(
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

    public boolean atState(LoaderConstants.LoaderState targetState) {
        return atVelocity(targetState.targetVelocityMetersPerSecond);
    }

    public boolean atTargetVelocity() {
        return atVelocity(targetVelocityMetersPerSecond);
    }

    public boolean atVelocity(double targetVelocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) <= LoaderConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    void feedToShooter() {
        final double targetShooterVelocityFromShootingCalculations = shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond();
        final double targetFeedingVelocity = targetShooterVelocityFromShootingCalculations * LoaderConstants.LOADING_TO_SHOOTER_COEFFICIENT;
        setTargetVelocity(targetFeedingVelocity);
    }

    void setTargetState(LoaderConstants.LoaderState targetState) {
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }
}
