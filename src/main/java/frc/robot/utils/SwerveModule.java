package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.*;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final int offset;

    public SwerveModule(int driveMotorPort, int angleMotorPort, int offset, boolean driveInverted,
            boolean angleInverted, boolean angleSensorPhase, double[] motionMagicConfigs) {
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonFX(angleMotorPort);
        this.offset = offset;

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kTalonTimeout);
        driveMotor.setInverted(driveInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.selectProfileSlot(1, 0);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, kTalonTimeout);
        angleMotor.configFeedbackNotContinuous(false, kTalonTimeout);
        angleMotor.setInverted(angleInverted);
        angleMotor.setSensorPhase(angleSensorPhase);

        configMotionMagic(motionMagicConfigs);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);
    }

    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[0], kTalonTimeout);
        angleMotor.config_kI(0, motionMagicConfigs[1], kTalonTimeout);
        angleMotor.config_kD(0, motionMagicConfigs[2], kTalonTimeout);
        angleMotor.config_kF(0, motionMagicConfigs[3], kTalonTimeout);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[4]);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[5]);
        angleMotor.configMotionAcceleration(motionMagicConfigs[6]);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[7]);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[8]);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[9]);
    }

    public Rotation2d toRoation2d(double ticks) {
        return new Rotation2d(ticks % kTicksPerRotationAngleMotor / (double) kTicksPerRotationAngleMotor * 2 * Math.PI);
    }

    public int toTicks(Rotation2d angle) {
        return (int) (angle.getRadians() / (2 * Math.PI) * kTicksPerRotationAngleMotor);
    }

    public Rotation2d getAngle() {
        return toRoation2d(angleMotor.getSelectedSensorPosition() - offset);
    }

    public void set(double speed, Rotation2d angle) {
        SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(speed, angle), getAngle());
        speed = optimized.speedMetersPerSecond;
        angle = optimized.angle;

        driveMotor.set(ControlMode.PercentOutput, speed);

        int error = toTicks(angle.minus(toRoation2d(angleMotor.getSelectedSensorPosition()))) + offset;
        angleMotor.set(ControlMode.MotionMagic, angleMotor.getSelectedSensorPosition() + error);
    }
}
