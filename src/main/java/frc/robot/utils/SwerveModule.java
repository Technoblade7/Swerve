package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

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
            boolean angleInverted, boolean angleSensorPhase, double[] anglePIDF) {
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
        angleMotor.config_kP(0, anglePIDF[0], kTalonTimeout);
        angleMotor.config_kI(0, anglePIDF[1], kTalonTimeout);
        angleMotor.config_kD(0, anglePIDF[2], kTalonTimeout);
        angleMotor.config_kF(0, anglePIDF[3], kTalonTimeout);
        angleMotor.config_IntegralZone(0, 5);
        angleMotor.configAllowableClosedloopError(0, toTicks(kAllowableAngleError));
        angleMotor.configMotionAcceleration(kAngleMotionAcceleration);
        angleMotor.configMotionCruiseVelocity(kAngleCruiseVelocity);
        angleMotor.configMotionSCurveStrength(kAngleCurveStrength);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(kNominalVoltage);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);
    }

    public Rotation2d toRoation2d(int ticks) {
        return new Rotation2d(ticks % kTicksPerRotationAngleMotor / (double) kTicksPerRotationAngleMotor * 2 * Math.PI);
    }

    public int toTicks(Rotation2d angle) {
        return (int) (angle.getRadians() / (2 * Math.PI) * kTicksPerRotationAngleMotor);
    }

    public void set(double speed, Rotation2d angle) {
        driveMotor.set(ControlMode.PercentOutput, speed);

        int error = toTicks(toRoation2d((int) angleMotor.getSelectedSensorPosition()).minus(angle)) - offset;
        angleMotor.set(ControlMode.MotionMagic, angleMotor.getSelectedSensorPosition() + error);
    }
}
