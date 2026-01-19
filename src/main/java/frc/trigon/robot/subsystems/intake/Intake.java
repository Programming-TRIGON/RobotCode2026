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
    private final TalonFXMotor angleMotor = IntakeConstants.ANGLE_MOTOR;
    private final TalonFXMotor intakeMotor = IntakeConstants.INTAKE_MOTOR;
    private final CANcoderEncoder angleEncoder = IntakeConstants.ANGLE_ENCODER;
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
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));

        /*log.motor("IntakeWheelMotor")
                .angularPosition(Units.Rotations.of(getWheelPosition().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVoltage()))
                .voltage(Units.Volts.of(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));*/
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.ANGLE_MOTOR_MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        IntakeConstants.WHEEL_MOTOR_MECHANISM.update(
                getCurrentVoltage()
        );
        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        angleMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        angleMotor.update();
        angleEncoder.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentAngle().getDegrees());
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
        intakeMotor.stopMotor();
    }

    public boolean atTargetState(IntakeConstants.IntakeState targetState) {
        return atAngle(targetState.targetAngle);
    }

    public void setTargetState(IntakeConstants.IntakeState targetState) {
        setTargetAngle(targetState);
        setTargetIntakeVoltage(targetState);
    }

    public void setTargetIntakeVoltage(IntakeConstants.IntakeState targetState) {
        intakeMotor.setControl(voltageRequest.withOutput(targetState.targetVoltage));
    }

    public void setTargetAngle(IntakeConstants.IntakeState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetVoltage(double targetVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(
                positionRequest.withPosition(targetAngle.getRotations())
        );
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                angleMotor.getSignal(TalonFXSignal.POSITION)
        );
    }

    private boolean atAngle(Rotation2d targetAngle) {
        return Math.abs(
                targetAngle.minus(getCurrentAngle()).getDegrees()
        ) < IntakeConstants.ANGLE_MOTOR_TOLERANCE.getDegrees();
    }

    private double getCurrentVoltage() {
        return intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0,0 ),
                new Rotation3d(0, getCurrentAngle().getRadians(), 0) //- check why i did -getCurrentAngle()
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }
}