// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.HangarLeft;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoConfig;
import frc.robot.commands.AutoExtendIntake;
import frc.robot.commands.AutoRetractIntake;
import frc.robot.commands.AutoRunFeeder;
import frc.robot.commands.AutoRunFeederAndTower;
import frc.robot.commands.AutoRunIntake;
import frc.robot.commands.AutoRunTower;
import frc.robot.commands.AutoSetShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.MoveWithRamsete;
import frc.robot.commands.QueueBalls;
import frc.robot.commands.RetractRunIntake;
import frc.robot.commands.StopDriveTrain;
import frc.robot.commands.StopWhenBallInBottomTower;
import frc.robot.commands.TurretAuto;
import frc.robot.commands.isRobotStopped;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class H_L_2BallEmergencyBall extends ParallelRaceGroup {

    Turret turret = Turret.getInstance();
    Limelight limelight = Limelight.getInstance();
    Intake intake = Intake.getInstance();
    DriveTrain driveTrain = DriveTrain.getInstance();

    private static final String ball2JSON = "paths/2ball.wpilib.json";

    public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.1, 1, new Rotation2d(-80)),
            AutoConfig.configFwdHigh);

    public static final Trajectory phase2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.1, 1, new Rotation2d(-80)),
            List.of(
                    new Translation2d(1.3, 2.5)),
            new Pose2d(1.7, 6, new Rotation2d(-55)),
            AutoConfig.configFwdHigh);

    public static final Trajectory phase3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 6, new Rotation2d(-55)),
            List.of(
                    new Translation2d(1.6, 4.5)),
            new Pose2d(0.5, 2, new Rotation2d(20)),
            AutoConfig.configRevHigh);

    /** Creates a new T_R_2ball. */
    public H_L_2BallEmergencyBall() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(

                new TurretAuto(limelight, turret),

                new AutoSetShoot(),

                //new WaitCommand(15),
                new SequentialCommandGroup(
                new ParallelRaceGroup(
                new SequentialCommandGroup(

                        new AutoExtendIntake(),

                        new ParallelRaceGroup(

                                new AutoRunIntake(),

                                new QueueBalls(true),

                                new MoveWithRamsete(phase1,
                                        driveTrain)
                                                .andThen(() -> driveTrain.tankDriveVolts(0, 0))

                        ),

                        new ParallelRaceGroup(
                                new AutoShoot(2),

                                new SequentialCommandGroup(
                                        new AutoRetractIntake(true),
                                        new AutoRunIntake())),

                        new AutoExtendIntake(),

                        new ParallelRaceGroup(
                                new MoveWithRamsete(phase2,
                                driveTrain)
                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                                new AutoRunIntake()

                        ),
                        
                                

                                new SequentialCommandGroup(
                                        //new AutoExtendIntake(),

                                        new ParallelRaceGroup(
                                                new QueueBalls(true),
                                                new AutoRunIntake(),
                                                //new WaitCommand(3),
                                                new StopWhenBallInBottomTower()),

                                        new ParallelCommandGroup(
                                                new AutoRetractIntake(),

                                        new MoveWithRamsete(phase3,
                                        driveTrain)
                                        .andThen(() -> driveTrain.tankDriveVolts(0, 0))
                                        

                ))),
                new WaitCommand(13).andThen(() -> System.out.println("Wait Command is not happening anymore"))
                ),
                //new StopDriveTrain(),
                new isRobotStopped(),
                new ParallelRaceGroup(
                        new RetractRunIntake(1),
                        //new AutoShoot(1)
                        new AutoRunFeederAndTower()
                )
                //new StopDriveTrain(),
                
                

        )
        );


    }

}
