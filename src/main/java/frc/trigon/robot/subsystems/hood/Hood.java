package frc.trigon.robot.subsystems.hood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = HoodConstants.MOTOR;
    private final CANcoderEncoder encoder = HoodConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(HoodConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(HoodConstants.FOC_ENABLED);
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Hood() {
        setName("Hood");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return HoodConstants.SYSID_CONFIG;
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
        log.motor("HoodMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        final Rotation2d currentAngle = getCurrentAngle();
        final Rotation2d targetProfiledAngle = Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
        HoodConstants.MECHANISM.update(
                currentAngle,
                targetProfiledAngle
        );
        Logger.recordOutput("Poses/Components/HoodPose", calculateVisualizationPose());

        Logger.recordOutput("Hood/TargetAngleDegrees", targetAngle.getDegrees());
        Logger.recordOutput("Hood/CurrentAngleDegrees", currentAngle.getDegrees());
        Logger.recordOutput("Hood/TargetProfiledAngleDegrees", targetProfiledAngle.getDegrees());
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        encoder.update();
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public boolean atTargetAngle() {
        return atAngle(targetAngle);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - getCurrentAngle().getDegrees()) < HoodConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION));
    }

    void aimAtHub() {
        final Rotation2d targetAngleFromShootingCalculations = shootingCalculations.getTargetShootingState().targetPitch();
        setTargetAngle(targetAngleFromShootingCalculations);
    }

    void aimForDelivery() {
        setTargetAngle(HoodConstants.DELIVERY_ANGLE);
    }

    void aimForEjection() {
        setTargetAngle(HoodConstants.EJECTION_ANGLE);
    }

    void rest() {
        setTargetAngle(HoodConstants.REST_ANGLE);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        motor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d turretPose = RobotContainer.TURRET.calculateVisualizationPose();
        final Transform3d hoodTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -getCurrentAngle().getRadians(), 0)
        );
        return turretPose.plus(HoodConstants.TURRET_TO_HOOD_OFFSET).plus(hoodTransform);
    }
}