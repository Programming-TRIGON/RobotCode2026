package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            motor = ClimberConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ClimberConstants.DEFAULT_MAXIMUM_VELOCITY,
            ClimberConstants.DEFAULT_MAXIMUM_ACCELERATION
    ).withEnableFOC(ClimberConstants.FOC_ENABLED);
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.REST;

    public Climber() {
        setName("Climber");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ClimberConstants.SYSID_CONFIG;
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
        log.motor("Climber")
                .angularPosition(Units.Rotations.of(getPositionRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ClimberConstants.MECHANISM.update(
                getPositionRotations(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );

        Logger.recordOutput("Poses/Components/ClimberPose", calculateComponentPose());
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }


    public boolean atTargetExtendedState() {
        return calculateTargetExtendedStateDistance() < ClimberConstants.POSITION_TOLERANCE_METERS;
    }

    void setTargetExtendedState(ClimberConstants.ClimberState targetState) {
        this.targetState = targetState;
        scalePositionRequestSpeed(targetState.extendedSpeedScalar);
        setTargetPositionRotations(metersToRotations(targetState.targetExtendedPositionMeters), ClimberConstants.CLIMBER_WEIGHT_SLOT);
    }

    void setTargetRetractedState() {
        scalePositionRequestSpeed(targetState.retractedSpeedScalar);
        setTargetPositionRotations(metersToRotations(targetState.targetRetractedPositionMeters), ClimberConstants.ROBOT_WEIGHT_SLOT);
    }

    void setTargetPositionRotations(double targetPositionRotations, int slot) {
        motor.setControl(positionRequest.withPosition(targetPositionRotations).withSlot(slot));
    }

    void resetPosition() {
        targetState = ClimberConstants.ClimberState.REST;
        motor.setControl(voltageRequest.withOutput(ClimberConstants.CLIMBER_RESET_VOLTAGE));
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = ClimberConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = ClimberConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private double calculateTargetExtendedStateDistance() {
        return Math.abs(targetState.targetExtendedPositionMeters - getPositionMeters());
    }

    private Pose3d calculateComponentPose() {
        return ClimberConstants.CLIMBER_VISUALIZATION_ORIGIN_POINT.transformBy(
                new Transform3d(
                        new Translation3d(0, 0, getPositionMeters()),
                        new Rotation3d()
                )
        );
    }

    private double metersToRotations(double positionMeters) {
        return Conversions.distanceToRotations(positionMeters, ClimberConstants.DRUM_DIAMETER_METERS);
    }

    private double getPositionMeters() {
        return rotationsToMeters(getPositionRotations());
    }


    private double getPositionRotations() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}