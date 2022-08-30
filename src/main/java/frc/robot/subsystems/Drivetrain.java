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

        private final SwerveModule mFl;
        private final SwerveModule mFr;
        private final SwerveModule mRl;
        private final SwerveModule mRr;

        private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public Drivetrain() {

                mFl = new SwerveModule(
                                kFrontLeftModuleDriveMotorId,
                                kFrontLeftModuleSteerMotorId,
                                kOffsets[0],
                                kFrontLeftDriveInverted,
                                kFrontLeftAngleInverted,
                                kFrontLeftAngleSensorPhase,
                                kFrontLeftAnglePIDF);

                mFr = new SwerveModule(
                                kFrontRightModuleDriveMotorId,
                                kFrontRightModuleSteerMotorId,
                                kOffsets[1],
                                kFrontRightDriveInverted,
                                kFrontRightAngleInverted,
                                kFrontRightAngleSensorPhase,
                                kFrontRightAnglePIDF);

                mRl = new SwerveModule(
                                kRearLeftModuleDriveMotorId,
                                kRearLeftModuleSteerMotorId,
                                kOffsets[2],
                                kRearLeftDriveInverted,
                                kRearLeftAngleInverted,
                                kRearLeftAngleSensorPhase,
                                kRearLeftAnglePIDF);

                mRr = new SwerveModule(
                                kRearRightModuleDriveMotorId,
                                kRearRightModuleSteerMotorId,
                                kOffsets[3],
                                kRearRightDriveInverted,
                                kRearRightAngleInverted,
                                kRearRightAngleSensorPhase,
                                kRearRightAnglePIDF);
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

        public void driveAutonomous(SwerveModuleState[] states) {
                mChassisSpeeds = mKinematics.toChassisSpeeds(states);
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxVelocityMetersPerSecond);

                mFl.set(states[0].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[0].angle);
                mFr.set(states[1].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[1].angle);
                mRl.set(states[2].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[2].angle);
                mRr.set(states[3].speedMetersPerSecond / kMaxVelocityMetersPerSecond,
                                states[3].angle);
        }
}