// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import static frc.robot.Constants.ClimberConstants.*;

/** Add your docs here. */
public class Climber extends SubsystemBase {

    private static TalonSRX erector;

    private static CANSparkMax motorLeft;
    private static CANSparkMax motorRight;
    
    public Climber() {
        erector = new TalonSRX(erectorMotorId);
        motorLeft = new CANSparkMax(motorLeftId, MotorType.kBrushless);
        motorRight = new CANSparkMax(motorRightId, MotorType.kBrushless);
        //I don't know what motors are being used for any of the three preceding types, but I will update once I find out.
    }

}
