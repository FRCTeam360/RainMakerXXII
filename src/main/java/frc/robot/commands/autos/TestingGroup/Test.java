// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.TestingGroup;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoSetShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.MoveWithRamsete;
import frc.robot.commands.TurretAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends ParallelRaceGroup {
    
  DriveTrain driveTrain = DriveTrain.getInstance();
  Limelight limelight = Limelight.getInstance();
  Turret turret = Turret.getInstance();

  public static final Trajectory test = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(2, 0, new Rotation2d(0)),
      AutoConfig.configFwd);

  public static final Trajectory test2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(0, 0, new Rotation2d(0)),
      AutoConfig.configRev);

  /** Creates a new Test. */
  public Test() {

    addCommands(
        new TurretAuto(limelight, turret),
        new AutoSetShoot(),
        new AutoShoot(2)
    );

    System.out.println("tessst");
    // addCommands();
  }
}
