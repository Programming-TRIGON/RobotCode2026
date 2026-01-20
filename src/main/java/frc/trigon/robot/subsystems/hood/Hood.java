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
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.turret.Turret;
import frc.trigon.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem {
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
        HoodConstants.MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + HoodConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET_ROTATION));
        Logger.recordOutput("Poses/Components/HoodPose", calculateVisualizationPose());
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

    void aimAtHub() {
    }//TODO implement

    void aimForDelivery() {
        setTargetAngle(HoodConstants.DELIVERY_ANGLE);
    }

    void rest() {
        setTargetAngle(HoodConstants.REST_ANGLE);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        final double offsettedTargetAngleRotations = targetAngle.getRotations() - HoodConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET_ROTATION;
        motor.setControl(positionRequest.withPosition(offsettedTargetAngleRotations));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d turretOrigin = TurretConstants.TURRET_VISUALIZATION_ORIGIN_POINT;
        final Pose3d turretPose = RobotContainer.TURRET.calculateVisualizationPose();
        final Pose3d hoodPoseAtTurretZeroRotation = new Pose3d(
                HoodConstants.HOOD_VISUALIZATION_ORIGIN_POINT.getTranslation(),
                new Rotation3d(0, getCurrentAngle().getRadians(), 0)
        );
        final Transform3d turretToPitcher = hoodPoseAtTurretZeroRotation.minus(turretOrigin);
        return turretPose.plus(turretToPitcher);
    }

    private Rotation2d getCurrentAngle() {
        final double offsettedCurrentAngleRotations = motor.getSignal(TalonFXSignal.POSITION) + HoodConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET_ROTATION;
        return Rotation2d.fromRotations(offsettedCurrentAngleRotations);
    }
}