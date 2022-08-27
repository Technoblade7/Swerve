package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final Drivetrain mDrivetrain;

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;

    public DefaultDrive(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        mDrivetrain = drivetrain;
        mTranslationXSupplier = translationXSupplier;
        mTranslationYSupplier = translationYSupplier;
        mRotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        mDrivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        mTranslationXSupplier.getAsDouble(),
                        mTranslationYSupplier.getAsDouble(),
                        mRotationSupplier.getAsDouble(),
                        mDrivetrain.getNavxRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
