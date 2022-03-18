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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.MoveWithRamsete;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test2 extends SequentialCommandGroup {
  
  DriveTrain driveTrain = DriveTrain.getInstance();

  public static final Trajectory test = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(2, 0),
          new Translation2d(2, -2)),
      new Pose2d(0, -2, new Rotation2d(180)),
      AutoConfig.configFwd);

  /** Creates a new Test. */
  public Test2() {

    addCommands(
        new MoveWithRamsete(
            test,
            driveTrain)
                .andThen(() -> driveTrain.tankDriveVolts(0, 0)));

    System.out.println("tessst");
    // addCommands();
  }
}
