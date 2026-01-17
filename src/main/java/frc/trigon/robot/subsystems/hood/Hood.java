package frc.trigon.robot.subsystems.hood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem {
    private final TalonFXMotor motor = HoodConstants.MOTOR;
    private final CANcoderEncoder angleEncoder = HoodConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(HoodConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(HoodConstants.FOC_ENABLED);

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
        log.motor("Hood")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        HoodConstants.MECHANISM.update(
                Rotation2d.fromRotations(getCurrentAngle().getRotations() + HoodConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET.getRotations()),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + HoodConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET.getRotations())
        );
        Logger.recordOutput("Poses/Components/HoodPose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        angleEncoder.update();
        Logger.recordOutput("Hood/CurrentPositionDegrees", getCurrentAngle().getDegrees());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }

    void aimAtHub() {
    }//TODO implement

    void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d hoodTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, getCurrentAngle().getRadians(), 0)//TODO implement turret rotation
        );
        return HoodConstants.HOOD_VISUALIZATION_ORIGIN_POINT.transformBy(hoodTransform);
    }
}