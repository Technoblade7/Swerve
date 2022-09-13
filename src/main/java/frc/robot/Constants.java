package frc.robot;

public final class Constants {
        public static final int kTalonTimeout = 10;
        public static final int kNominalVoltage = 12;

        // Swerve
        public static final int[] kOffsets = { -865, -606, -703, -870 };

        public static final double kDrivetrainTrackwidthMeters = 0.5224;
        public static final double kDrivetrainWheelbaseMeters = 0.6624;

        public static final int kFrontLeftModuleDriveMotorId = 7;
        public static final int kFrontLeftModuleSteerMotorId = 8;
        public static boolean kFrontLeftDriveInverted = false;
        public static boolean kFrontLeftAngleInverted = true;
        public static boolean kFrontLeftAngleSensorPhase = false;

        public static final int kFrontRightModuleDriveMotorId = 3;
        public static final int kFrontRightModuleSteerMotorId = 4;
        public static boolean kFrontRightDriveInverted = false;
        public static boolean kFrontRightAngleInverted = true;
        public static boolean kFrontRightAngleSensorPhase = false;

        public static final int kRearLeftModuleDriveMotorId = 1;
        public static final int kRearLeftModuleSteerMotorId = 6;
        public static boolean kRearLeftDriveInverted = false;
        public static boolean kRearLeftAngleInverted = true;
        public static boolean kRearLeftAngleSensorPhase = false;

        public static final int kRearRightModuleDriveMotorId = 5;
        public static final int kRearRightModuleSteerMotorId = 2;
        public static boolean kRearRightDriveInverted = false;
        public static boolean kRearRightAngleInverted = true;
        public static boolean kRearRightAngleSensorPhase = false;

        public static final double kDriveReduction = 0.1;
        public static final double kWheelDiameter = 0.1;

        public static final int kTicksPerRotationAngleMotor = 1024;

        // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
        // maxIntegralAccum, peakOutput
        public static final double[] kFrontLeftMotionMagicConfigs = { 2, 0, 0, 0, 1, 400, 1300, 10, 5, 1 };
        public static final double[] kFrontRightMotionMagicConfigs = { 2, 0, 0, 0, 1, 400, 1300, 10, 5, 1 };
        public static final double[] kRearLeftMotionMagicConfigs = { 2, 0, 0, 0, 1, 400, 1300, 10, 5, 1 };
        public static final double[] kRearRightMotionMagicConfigs = { 2, 0, 0, 0, 1, 400, 1300, 10, 5, 1 };

        public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 *
                        kDriveReduction *
                        kWheelDiameter * Math.PI;
        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
                        Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

}
