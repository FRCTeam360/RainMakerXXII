// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.TestingGroup.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.autos.Anywhere.Anywhere2ball;
import frc.robot.commands.autos.HangarLeft.H_L_2ball;
import frc.robot.commands.autos.HangarLeft.H_L_Test;
import frc.robot.commands.autos.HangarLeft.SimpleHL2ball;
import frc.robot.commands.autos.TerminalLeft.SimpleTL2ball;
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
    private final Command hl2ball;
    private final Command hltest;
    private final Command anywhere2ball;
    private final Command simplehl;
    private final Command simpletl;
    private final Command simpletr;
    private final Command trTest;

    public AutoChooser(RobotContainer container) {

        selectedLocation = "None";

        locationChooser = new SendableChooser<>();
        autoChooser = new SendableChooser<>();

        test = new Test();
        test2 = new Test2();
        tr2ball = new T_R_2ball();
        trTest = new T_R_Test();
        hl2ball = new H_L_2ball();
        hltest = new H_L_Test();
        anywhere2ball = new Anywhere2ball(0);
        simplehl = new SimpleHL2ball();
        simpletl = new SimpleTL2ball();
        simpletr = new SimpleTR2ball();

        // locationChooser.addOption("Test", "Test");
        locationChooser.addOption("Hangar Left", "Hangar Left");
        locationChooser.addOption("Hangar Center", "Hangar Center");
        locationChooser.addOption("Hanger Right", "Hangar Right");
        locationChooser.addOption("Terminal Left", "Terminal Left");
        locationChooser.addOption("Terminal Center", "Terminal Center");
        locationChooser.addOption("Terminal Right", "Terminal Right");
        locationChooser.addOption("Test", "Test");
        locationChooser.setDefaultOption("Anywhere", "Anywhere");

        SmartDashboard.putData("Start Location", locationChooser);
        SmartDashboard.putData("Auto Choice", autoChooser);
    }

    public void periodic() {
        if (selectedLocation != null && !selectedLocation.equals(locationChooser.getSelected())) { // If it changes or
                                                                                                   // is being
                                                                                                   // initialized

            selectedLocation = locationChooser.getSelected(); // Reset the SelectedLocation to what it actually is
            autoChooser = new SendableChooser<>(); // Clear the auto chooser

            if ("Test".equals(selectedLocation)) {
                autoChooser.addOption("Test", test);
                autoChooser.addOption("Test2", test2);
            }

            if ("Anywhere".equals(selectedLocation)) {
                autoChooser.addOption("Simple anywhere", anywhere2ball);
            }

            if ("Terminal Right".equals(selectedLocation)) {
                autoChooser.addOption("TR 2 ball", tr2ball);
                autoChooser.addOption("Simple TR", simpletr);
                autoChooser.addOption("TR Test", trTest);
            }

            if ("Hangar Left".equals(selectedLocation)) {
                autoChooser.addOption("HL 2 ball", hl2ball);
                autoChooser.addOption("Simple HL", simplehl);
                autoChooser.addOption("HL Test", hltest);
            }

            if ("Terminal Right".equals(selectedLocation)) {
                autoChooser.addOption("Simple TR", simpletr);
            }

            SmartDashboard.putData("Auto Choice", autoChooser); // Update the Auto Choice with the new options and new
                                                                // chooser

        }
        // Else do nothing cuz the location chooser hasn't changed states
    }

    public Command getCommand() {
        return autoChooser.getSelected();
    }
}
