package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor AngleMotor = IntakeConstants.ANGLE_MOTOR;
    private final TalonFXMotor WheelMotor = IntakeConstants.WHEEL_MOTOR;
    private final CANcoderEncoder AngleEncoder = IntakeConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IntakeAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(AngleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(AngleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("IntakeWheelMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityRotationsPerSecond()))
                .voltage(Units.Volts.of(WheelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_ANGLE_MOTOR_MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(AngleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        IntakeConstants.INTAKE_WHEEL_MOTOR_MECHANISM.update(
                getCurrentVelocityRotationsPerSecond(),
                WheelMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );

        //Logger.recordOutput("Poses/Components/IntakePose", getComponentPose()); pose3d
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        AngleMotor.setControl(voltageRequest.withOutput(targetVoltage));
        WheelMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        AngleMotor.setBrake(brake);
        WheelMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        AngleMotor.update();
        AngleEncoder.update();
        WheelMotor.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentAngle().getDegrees());
        //Logger.recordOutput(); for the wheel ask Yishay
    }

    @Override
    public void stop() {
        AngleMotor.stopMotor();
    }

    public boolean atTargetAngle() {
        return atAngle(targetAngle);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(
                angle.minus(getCurrentAngle()).getDegrees()
        ) < IntakeConstants.INTAKE_WHEEL_MOTOR_TOLERANCE.getDegrees();
    }

    public void AngleMotorSetTargetState(IntakeConstants.AngleMotorState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    public void WheelMotorSetTargetState(IntakeConstants.WheelMotorState targetState) {
        voltageRequest.withOutput(targetState.targetVoltage);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        AngleMotor.setControl(
                positionRequest.withPosition(targetAngle.getRotations())
        );
    }

    public void setTargetVoltage(double targetVoltage) {
        voltageRequest.withOutput(targetVoltage);
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                AngleMotor.getSignal(TalonFXSignal.POSITION)
        );
    }

    private double getCurrentVelocityRotationsPerSecond() {
        return WheelMotor.getSignal(TalonFXSignal.VELOCITY);
    }

    private Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(WheelMotor.getSignal(TalonFXSignal.POSITION));
    }
}
