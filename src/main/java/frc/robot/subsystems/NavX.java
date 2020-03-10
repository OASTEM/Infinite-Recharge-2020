/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */
  private AHRS navX;
  Timer timer;
  double posMax = 0;
  double negMax = 0;  

  public NavX() {
    navX = new AHRS(Port.kMXP, (byte)50);
    timer = new Timer();
    timer.start();
    //refer to rio for xyz not navx

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(timer.get() + ": " + navX.getVelocityY());
    //System.out.println(navX.getWorldLinearAccelY());
    /*if (Math.abs(getAccelX()) < .7) {
      System.out.println(getAccelX());
      System.out.println("Velocity" + navX.getVelocityY());
      System.out.println("Accel: " + navX.getWorldLinearAccelY());
    }*/
    
    if (getAccelX() > posMax) {
      posMax = getAccelX();
    }
    else if (getAccelX() < negMax) {
      negMax = getAccelX();
    }
    //System.out.println("Accel X: " + getAccelX());
    //System.out.println("Positive Max: " + posMax);
    //System.out.println("Negative Max: " + negMax);
  }

  public double getAccelX() {
    return navX.getRawAccelX();
  }

  public double getAccelY() {
    return navX.getRawAccelY();
  }

  public double getAccelZ() {
    return navX.getRawAccelZ();
  }

  public double getGyroY() {
    return navX.getRawGyroY();
  }

  public double getGyroX() {
    return navX.getRawGyroX();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public void reset() {
    navX.reset();
    posMax = 0;
    negMax = 0;
  }
  
  public double getLinearAccelY() {
    return navX.getWorldLinearAccelY();
  }
}



