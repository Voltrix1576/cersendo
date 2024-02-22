package frc.robot.commands.Autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class autoLine extends SequentialCommandGroup {
    public autoLine(SwerveDrivetrain swerve) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                SwerveConstants.maxV,
                SwerveConstants.maxAcceleration)
            .setKinematics(swerve.getInstance().getKinematics());

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0),new Translation2d(2, 0),new Translation2d(3, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            1, 0, 0, SwerveConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            swerve::getPose,
            swerve.getInstance().getKinematics(),
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController,
            swerve::setSwerveState,
            swerve);


    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
    }

