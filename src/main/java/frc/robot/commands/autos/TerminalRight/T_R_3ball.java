// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.TerminalRight;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.Timer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.commands.AutoExtendIntake;
import frc.robot.commands.AutoFeedBall;
import frc.robot.commands.AutoRetractIntake;
import frc.robot.commands.AutoRetractIntake;
import frc.robot.commands.AutoRunFeeder;
import frc.robot.commands.AutoRunFeederAndTower;
import frc.robot.commands.AutoRunIntake;
import frc.robot.commands.AutoRunTower;
import frc.robot.commands.AutoSetShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTimer;
import frc.robot.commands.MoveWithRamsete;
import frc.robot.commands.QueueBalls;
import frc.robot.commands.QueueBalls;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.TurretAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class T_R_3ball extends ParallelRaceGroup {

    Turret turret = Turret.getInstance();
    Limelight limelight = Limelight.getInstance();
    Intake intake = Intake.getInstance();
    DriveTrain driveTrain = DriveTrain.getInstance();

    private static final String ball2JSON = "paths/2ball.wpilib.json";
    // Trajectory phase1 = new Trajectory();

    public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.1, -1, new Rotation2d(-90)),
            AutoConfig.configFwd);

    public static final Trajectory phase2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.1, -1, new Rotation2d(-90)),
            List.of(
                new Translation2d(0, -2)
            ),
            new Pose2d(0, -2.75, new Rotation2d(-45)),
            AutoConfig.configFwd);

    /** Creates a new T_R_2ball. */
    public T_R_3ball() {

        addCommands(

                new TurretAuto(limelight, turret),

                new AutoSetShoot(),

                new SequentialCommandGroup(

                        new SequentialCommandGroup(

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

                        ),

                        new SequentialCommandGroup(


                                new AutoExtendIntake(),

                                new ParallelRaceGroup(

                                        new AutoRunIntake(),

                                        new QueueBalls(true),

                                        new MoveWithRamsete(phase2,
                                                driveTrain)
                                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0))

                                ),

                                new AutoRetractIntake(true),

                                new AutoShoot(1)

                                )));
    }

}