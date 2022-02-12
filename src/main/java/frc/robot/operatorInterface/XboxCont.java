/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operatorInterface;

import edu.wpi.first.wpilibj.XboxController;

public class XboxCont extends XboxController {
  /**
   * Creates a new XboxCont.
   */
  public XboxCont(int port) {
    super(port);
  }

  public boolean getLeftTrigger() {
    return getLeftTriggerAxis() > 0.1;
  }

  public boolean getRightTrigger() {
    return getRightTriggerAxis() > 0.1;
  }

  public boolean getDPadUp(){
    return getPOV() == 0;
  }

  public boolean getDPadRight(){
    return getPOV() == 90;
  }

  public boolean getDPadDown(){
    return getPOV() == 180;
  }

  public boolean getDPadLeft(){
    return getPOV() == 270;
  }
}
