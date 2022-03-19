// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class CargoCounter {

  private Integer cargoCount = 0, shotCount = 0;
  private static CargoCounter instance;


  /** Creates a new BallCounter. */
  private CargoCounter() {
  }

  public static CargoCounter getInstance() {
    if (instance == null) {
      instance = new CargoCounter();
    }
    return instance;
  }

  public void addBallToCount() {
    cargoCount += 1;
  }

  public void subtractBallFromCount() {
    if (cargoCount > 0) {
      cargoCount -= 1;
    }
  }

  public Integer getQueuedCount() {
    return this.cargoCount;
  }

  public void resetCargoCount() {
    cargoCount = 0;
  }

  public Integer getShotCount(){
    return this.shotCount;
  }

  public void incrementShootCount(){
    shotCount += 1;

    if(cargoCount > 0){
      cargoCount -= 1;
    }

    System.out.println("shot registed, shot " + shotCount + " of balls in total");
  }
}
