package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

public class RobotContainer {
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final XboxController mController = new XboxController(0);

  public RobotContainer() {
    mDrivetrain.setDefaultCommand(
        new DefaultDrive(mDrivetrain,
            () -> -modifyAxis(mController.getLeftY()) * kMaxVelocityMetersPerSecond,
            () -> -modifyAxis(mController.getLeftX()) * kMaxVelocityMetersPerSecond,
            () -> -modifyAxis(mController.getRightX()) * kMaxAngularVelocityRadiansPerSecond));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(mController::getAButton).whenPressed(mDrivetrain::zeroNavx);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
