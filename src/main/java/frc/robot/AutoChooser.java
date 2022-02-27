// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.TestingGroup.*;
import frc.robot.commands.autos.TerminalRight.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {

    private SendableChooser<String> locationChooser;
    private SendableChooser<Command> autoChooser;

    private String selectedLocation;

    private final Command test;
    private final Command test2;
    private final Command tr2ball;

    public AutoChooser(RobotContainer container){
        
        selectedLocation = "None";

        locationChooser = new SendableChooser<>();
        autoChooser = new SendableChooser<>();

        test = new Test(container.getDriveTrain());
        test2 = new Test2(container.getDriveTrain());
        tr2ball = new T_R_2ball(container.getDriveTrain());

        locationChooser.addOption("Test", "Test");
        locationChooser.addOption("Hangar Left", "Hangar Left");
        locationChooser.addOption("Hangar Center", "Hangar Center");
        locationChooser.addOption("Hanger Right", "Hangar Right");
        locationChooser.addOption("Terminal Left", "Terminal Left");
        locationChooser.addOption("Terminal Center", "Terminal Center");
        locationChooser.addOption("Terminal Right", "Terminal Right");

        SmartDashboard.putData("Start Location", locationChooser);
        SmartDashboard.putData("Auto Choice", autoChooser);
        System.out.println("working");
        autoChooser.setDefaultOption("Test", test2);
    }

    public void periodic() {
        if ( !selectedLocation.equals( locationChooser.getSelected() ) ) { //If it changes or is being initialized

            selectedLocation = locationChooser.getSelected(); //Reset the SelectedLocation to what it actually is 
            autoChooser = new SendableChooser<>(); //Clear the auto chooser

            if (selectedLocation.equals("Test")) {
                autoChooser.addOption("Test", test);
            }

            if (selectedLocation.equals("Terminal Right")) {
                autoChooser.addOption("2 ball", tr2ball);
            }

            SmartDashboard.putData(autoChooser); //Update the Auto Choice with the new options and new chooser

        }
        //Else do nothing cuz the location chooser hasn't changed states
    }

    public Command getCommand() {
        return autoChooser.getSelected();
    }
}
