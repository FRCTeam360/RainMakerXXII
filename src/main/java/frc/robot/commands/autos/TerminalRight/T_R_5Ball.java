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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoConfig;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class T_R_5Ball extends ParallelRaceGroup {

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
            AutoConfig.configFwdHigh);

    public static final Trajectory phase11 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.1, -1, new Rotation2d(-90)),
            List.of(
                    new Translation2d(.25, -3)),
            new Pose2d(0, -3.5, new Rotation2d(-90)),
            AutoConfig.configFwdHigh);

    public static final Trajectory phase2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, -3.5, new Rotation2d(-90)),
            List.of(
            // new Translation2d(0, -7)),
            ),
            new Pose2d(0.60 + AutoConstants.getXOffsetTerminal5Ball(), -7 + AutoConstants.getYOffsetTerminal5Ball(), new Rotation2d(-45)),
            AutoConfig.configFwdHigh);

    private static final Trajectory phase3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.60 + AutoConstants.getXOffsetTerminal5Ball(), -7 + AutoConstants.getYOffsetTerminal5Ball(), new Rotation2d(-45)),
            List.of(),
            new Pose2d(0.5 + AutoConstants.getXOffsetTerminal5Ball(), -6.9 + AutoConstants.getYOffsetTerminal5Ball(), new Rotation2d(-45)),
            AutoConfig.configRevHigh);
            
    private static final Trajectory phase4 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.5 + AutoConstants.getXOffsetTerminal5Ball(), -6.9 + AutoConstants.getYOffsetTerminal5Ball(), new Rotation2d(-45)),
            List.of(),
            new Pose2d(0, -3.5, new Rotation2d(-45)),
            AutoConfig.configRevHigh);

    /** Creates a new T_R_2ball. */
    public T_R_5Ball() {
        addCommands(
                new TurretAuto(limelight, turret),
                new AutoSetShoot(),
                new PrintTime(),
                new TakeSnapshots(),
                // new WaitCommand(15),

                new SequentialCommandGroup(
                    new InstantCommand(() -> driveTrain.setDriveOffset(1.5)),
                        // new AutoRunFeederAndTower(),
                        new AutoExtendIntake(),
                        // new AutoShoot(1),
                        new ParallelRaceGroup(
                                new MoveWithRamsete(phase1,
                                        driveTrain)
                                                .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                                new AutoRunIntake(),
                                new QueueBalls(true)),
                        new AutoRetractIntake(),
                        new ParallelRaceGroup(
                                new AutoRunIntake(),
                                new AutoShoot(2)
                        ),
                        new ParallelRaceGroup(
                                new AutoRunIntake(),
                                new MoveWithRamsete(phase11,
                                        driveTrain)
                                                .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                                new QueueBalls(true)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new AutoRetractIntake(true),
                                        new AutoRunIntake()),
                                new AutoShoot(1)),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new AutoExtendIntake(),
                                        new AutoRunIntake()),
                                new QueueBalls(true),
                                new SequentialCommandGroup(
                                        new MoveWithRamsete(phase2,
                                                driveTrain)
                                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                                        new MoveWithRamsete(phase3,
                                                driveTrain)
                                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                                        new WaitCommand(0.5),
                                        new MoveWithRamsete(phase4,
                                                driveTrain)
                                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0)))

                        ),

                        new ParallelCommandGroup(
                                new AutoRetractIntake(true),
                                new AutoShoot(2))
                                )

        );
    }

}
