package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
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
        ShooterConstants.FOLLOWER_MOTOR.update();
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
    public void updateMechanism() {
        final double currentVelocityMetersPerSecond = getCurrentVelocityMetersPerSecond();
        final double targetProfiledVelocityMetersPerSecond = motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE);
        ShooterConstants.MECHANISM.update(
                currentVelocityMetersPerSecond,
                targetProfiledVelocityMetersPerSecond
        );

        Logger.recordOutput("Shooter/CurrentVelocityMetersPerSecond", currentVelocityMetersPerSecond);
        Logger.recordOutput("Shooter/TargetVelocityMetersPerSecond", this.targetVelocityMetersPerSecond);
        Logger.recordOutput("Shooter/TargetProfiledVelocityMetersPerSecond", targetProfiledVelocityMetersPerSecond);
    }

    public boolean atTargetVelocity() {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) < ShooterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    public boolean isAimingAtHub() {
        return targetVelocityMetersPerSecond == shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond();
    }

    public double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    void aimAtHub() {
        final double targetVelocityFromShootingCalculations = shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond();
        final double targetVelocityWithSlippageCompensation = targetVelocityFromShootingCalculations * ShooterConstants.WHEEL_SLIPPAGE_COMPENSATION_VELOCITY_MULTIPLIER;

        setTargetVelocity(targetVelocityWithSlippageCompensation);
    }

    void aimForDelivery() {
        setTargetVelocity(calculateDeliveryShootingVelocity());
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    private double calculateDeliveryShootingVelocity() {
        final double currentXVelocity = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
        final double distanceToDeliveryPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getDistance(RobotContainer.TURRET.calculateClosestDeliveryPosition());
        final double distanceAimingVelocity = (distanceToDeliveryPosition * ShooterConstants.DELIVERY_VELOCITY_SLOPE) + ShooterConstants.DELIVERY_VELOCITY_INTERCEPT_POINT;
        final double currentXVelocityTowardsAlliance = Flippable.isRedAlliance() ? -currentXVelocity : currentXVelocity;
        return distanceAimingVelocity + currentXVelocityTowardsAlliance;
    }
}