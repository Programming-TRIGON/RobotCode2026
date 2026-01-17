package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final CANcoderEncoder AngleEncoder = IntakeConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Intake() {
        setName(IntakeConstants.MECHANISM_NAME);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(AngleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(AngleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(AngleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        AngleMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        AngleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        AngleMotor.update();
        AngleEncoder.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentAngle().getDegrees());
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
        ) < IntakeConstants.TOLERANCE.getDegrees();
    }

    public void setTargetState(IntakeConstants.AngleMotorState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        AngleMotor.setControl(
                positionRequest.withPosition(targetAngle.getRotations())
        );
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                AngleMotor.getSignal(TalonFXSignal.POSITION)
        );
    }

    public void isTheTriggerWork(){
        System.out.println("working");
    };
}
