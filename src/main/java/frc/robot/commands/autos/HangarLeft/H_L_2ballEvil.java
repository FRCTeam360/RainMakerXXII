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
import frc.robot.commands.AutoExtendIntake;
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
import frc.robot.commands.RunFeeder;
import frc.robot.commands.TurretAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class H_L_2ballEvil extends ParallelRaceGroup {

  Turret turret = Turret.getInstance();
  Limelight limelight = Limelight.getInstance();
  Intake intake = Intake.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();

  private static final String ball2JSON = "paths/2ball.wpilib.json";
  // Trajectory phase1 = new Trajectory();

  public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1, 0, new Rotation2d(0)),
      AutoConfig.configFwdLow);

  public static final Trajectory phase2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.4, -1, new Rotation2d(0)),
      AutoConfig.configFwdLow);

  /** Creates a new T_R_2ball. */
  public H_L_2ballEvil() {

    addCommands(

        new TurretAuto(limelight, turret),

        new AutoSetShoot(),

        new SequentialCommandGroup(

            new InstantCommand(() -> driveTrain.setDriveOffset(133.5)),

            new AutoExtendIntake(),

            new ParallelRaceGroup(

                new AutoRunIntake(),

                new QueueBalls(true),

                new MoveWithRamsete(phase1, 
                driveTrain)
                .andThen(() -> driveTrain.tankDriveVolts(0, 0))

            ),

            new AutoRetractIntake(),
            
            new AutoShoot(2),

            new MoveWithRamsete(phase2, 
                driveTrain)
                .andThen(() -> driveTrain.tankDriveVolts(0, 0))
        )
    );
  }
}
