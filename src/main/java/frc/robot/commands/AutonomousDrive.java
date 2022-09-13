package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

class AutotonomousDrive extends PPSwerveControllerCommand {

    public AutotonomousDrive(Drivetrain drivetrain, String pathName) {
        super(PathPlanner.loadPath(pathName, 8, 5),
                drivetrain::getPose,
                drivetrain.getKinematics(),
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                new ProfiledPIDController(2, 0, 0,
                        new TrapezoidProfile.Constraints(kMaxAngularVelocityRadiansPerSecond,
                                kMaxAngularVelocityRadiansPerSecond / 2)),
                states -> drivetrain.setStates(states),
                drivetrain);
    }
}