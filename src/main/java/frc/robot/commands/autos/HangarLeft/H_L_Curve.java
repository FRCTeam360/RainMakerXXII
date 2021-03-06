// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.HangarLeft;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.Timer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class H_L_Curve extends ParallelRaceGroup {

    Turret turret = Turret.getInstance();
    Limelight limelight = Limelight.getInstance();
    Intake intake = Intake.getInstance();
    DriveTrain driveTrain = DriveTrain.getInstance();

    private static final String ball2JSON = "paths/2ball.wpilib.json";
    // Trajectory phase1 = new Trajectory();

    public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.1, 1, new Rotation2d(90)),
            AutoConfig.configFwdLow);

  /** Creates a new T_R_2ball. */
  public H_L_Curve() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    

    // try {
    // Path ball2 = Filesystem.getDeployDirectory().toPath().resolve(ball2JSON);
    // phase1 = TrajectoryUtil.fromPathweaverJson(ball2);
    // } catch (IOException ex) {
    // DriverStation.reportError("Unable to open trajectory: " + ball2JSON,
    // ex.getStackTrace());
    // }

    // driveTrain.angleAdjust(133.5);

    addCommands(

        new TurretAuto(limelight, turret),

        new AutoSetShoot(),

        new SequentialCommandGroup(

            new InstantCommand(() -> driveTrain.setDriveOffset(133.5)),

            // new AutoRunFeederAndTower(),

            new AutoExtendIntake(),

            new ParallelRaceGroup(

                new AutoRunIntake(),

                new QueueBalls(true),

                new MoveWithRamsete(phase1, 
                driveTrain)
                .andThen(() -> driveTrain.tankDriveVolts(0, 0))

            ), 

            new AutoRetractIntake(true),

            new AutoShoot(2)
        )
    );

  }
}
