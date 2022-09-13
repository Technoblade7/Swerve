package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SwerveModule;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
        private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
                        // Front right
                        new Translation2d(kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0),
                        // Rear left
                        new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
                        // Rear right
                        new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0));

        private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(),
                        new Pose2d());

        private final AHRS mNavx = new AHRS();

        private final SwerveModule mFrontLeft;
        private final SwerveModule mFrontRight;
        private final SwerveModule mRearLeft;
        private final SwerveModule mRearRight;

        private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public Drivetrain() {

                mFrontLeft = new SwerveModule(
                                kFrontLeftModuleDriveMotorId,
                                kFrontLeftModuleSteerMotorId,
                                kOffsets[0],
                                kFrontLeftDriveInverted,
                                kFrontLeftAngleInverted,
                                kFrontLeftAngleSensorPhase,
                                kFrontLeftMotionMagicConfigs);

                mFrontRight = new SwerveModule(
                                kFrontRightModuleDriveMotorId,
                                kFrontRightModuleSteerMotorId,
                                kOffsets[1],
                                kFrontRightDriveInverted,
                                kFrontRightAngleInverted,
                                kFrontRightAngleSensorPhase,
                                kFrontRightMotionMagicConfigs);

                mRearLeft = new SwerveModule(
                                kRearLeftModuleDriveMotorId,
                                kRearLeftModuleSteerMotorId,
                                kOffsets[2],
                                kRearLeftDriveInverted,
                                kRearLeftAngleInverted,
                                kRearLeftAngleSensorPhase,
                                kRearLeftMotionMagicConfigs);

                mRearRight = new SwerveModule(
                                kRearRightModuleDriveMotorId,
                                kRearRightModuleSteerMotorId,
                                kOffsets[3],
                                kRearRightDriveInverted,
                                kRearRightAngleInverted,
                                kRearRightAngleSensorPhase,
                                kRearRightMotionMagicConfigs);
        }

        public void zeroNavx() {
                mNavx.reset();
        }

        public Rotation2d getNavxRotation() {
                return Rotation2d.fromDegrees(mNavx.getYaw());
        }

        public SwerveDriveKinematics getKinematics() {
                return mKinematics;
        }

        public void updateOdometry() {
                mOdometry.update(getNavxRotation(), mKinematics.toSwerveModuleStates(mChassisSpeeds));
        }

        public Pose2d getPose() {
                return mOdometry.getPoseMeters();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                mChassisSpeeds = chassisSpeeds;
        }

        public void setStates(SwerveModuleState[] states) {
                mChassisSpeeds = mKinematics.toChassisSpeeds(states);
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxVelocityMetersPerSecond);

                mFrontLeft.set(states[0].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[0].angle);
                mFrontRight.set(states[1].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[1].angle);
                mRearLeft.set(states[2].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[2].angle);
                mRearRight.set(states[3].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[3].angle);
        }
}