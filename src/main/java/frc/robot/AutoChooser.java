// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.TestingGroup.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {

    private SendableChooser<String> locationChooser;
    private SendableChooser<Command> autoChooser;

    private String selectedLocation;

    private final Command test;

    public AutoChooser(RobotContainer container){
        
        selectedLocation = "None";

        locationChooser = new SendableChooser<>();
        autoChooser = new SendableChooser<>();

        test = new Test(container.driveTrain);
    }
}
