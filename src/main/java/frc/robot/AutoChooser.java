// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.autos.TestingGroup.*;
import frc.robot.commands.autos.Anywhere.*;
import frc.robot.commands.autos.HangarLeft.*;
import frc.robot.commands.autos.TerminalLeft.*;
import frc.robot.commands.autos.TerminalRight.*;


public class AutoChooser {

    // private SendableChooser<String> locationChooser;
    private SendableChooser<Command> autoChooser;

    // private String selectedLocation;

    private final Command anywhere2Ball;

    private final Command hangarLeft2Ball;
    private final Command hangarLeft2BallCurve;
    private final Command simpleHangarLeft;

    private final Command simpleTerminalLeft;

    private final Command simpleTerminalRight;
    private final Command terminalRight2Ball;
    private final Command terminalRight3Ball;
    private final Command terminalRight5Ball;

    private final Command test;
    private final Command test2;

    public AutoChooser(RobotContainer container) {

        // selectedLocation = "None";

        // locationChooser = new SendableChooser<>();
        autoChooser = new SendableChooser<>();

        anywhere2Ball = new Anywhere2ball(0);

        hangarLeft2Ball = new H_L_2ball();
        hangarLeft2BallCurve = new H_L_Curve();
        simpleHangarLeft = new SimpleHL2ball();

        simpleTerminalLeft = new SimpleTL2ball();
        
        simpleTerminalRight = new SimpleTR2ball();
        terminalRight2Ball = new T_R_2ball();
        terminalRight3Ball = new T_R_3ball();
        terminalRight5Ball = new T_R_5Ball();

        test = new Test();
        test2 = new Test2();

        // locationChooser.addOption("Test", "Test");
        // locationChooser.addOption("Hangar Left", "Hangar Left");
        // locationChooser.addOption("Hangar Center", "Hangar Center");
        // locationChooser.addOption("Hanger Right", "Hangar Right");
        // locationChooser.addOption("Terminal Left", "Terminal Left");
        // locationChooser.addOption("Terminal Center", "Terminal Center");
        // locationChooser.addOption("Terminal Right", "Terminal Right");
        // locationChooser.addOption("Test", "Test");
        // locationChooser.setDefaultOption("Anywhere", "Anywhere");

        // SmartDashboard.putData("Start Location", locationChooser);
        SmartDashboard.putData("Auto Choice", autoChooser);
    }

    public void periodic() {
        // if (selectedLocation != null && !selectedLocation.equals(locationChooser.getSelected())) { // If it changes or
                                                                                                   // is being
                                                                                                   // initialized

            // selectedLocation = locationChooser.getSelected(); // Reset the SelectedLocation to what it actually is
            autoChooser = new SendableChooser<>(); // Clear the auto chooser

            // if ("Test".equals(selectedLocation)) {
            //     autoChooser.addOption("Test", test);
            //     autoChooser.addOption("Test2", test2);
            // }

            // if ("Anywhere".equals(selectedLocation)) {
                autoChooser.addOption("Simple anywhere", anywhere2Ball);
            // }

            // if ("Terminal Right".equals(selectedLocation)) {
                // autoChooser.addOption("Simple TR", simpleTerminalRight);
                autoChooser.addOption("TR 2 ball", terminalRight2Ball);
                autoChooser.addOption("TR 3 ball", terminalRight3Ball);
                autoChooser.addOption("TR 5 ball", terminalRight5Ball);
            // }

            // if ("Hangar Left".equals(selectedLocation)) {
                // autoChooser.addOption("Simple HL", simpleHangarLeft);
                autoChooser.addOption("HL 2 ball", hangarLeft2Ball);
                autoChooser.addOption("HL 2 ball curve", hangarLeft2BallCurve);
            // }

            // if ("Terminal Right".equals(selectedLocation)) {
                // autoChooser.addOption("Simple TR", simpleTerminalRight);
            // }

            SmartDashboard.putData("Auto Choice", autoChooser); // Update the Auto Choice with the new options and new
                                                                // chooser

        }
        // Else do nothing cuz the location chooser hasn't changed states
    // }

    public Command getCommand() {
        return autoChooser.getSelected();
    }
}
