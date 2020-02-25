/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnAngle extends CommandBase {
  /**
   * Creates a new TurnAngle.
   */
  private double angle;
  Timer timer = new Timer();
  public TurnAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.navX);
    addRequirements(RobotContainer.drive);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navX.reset();
    Timer.delay(.1);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = RobotContainer.navX.getAngle();
    System.out.println(currentAngle);
    double power = ((angle - Math.abs(currentAngle))/angle)*(angle*0.0146);
    if (angle > 0) {
      if (power > .15) {
        RobotContainer.drive.drivePercentOutput(-.15, .15);
        System.out.println("first condition");
      } else {
        System.out.println("else");
        RobotContainer.drive.drivePercentOutput(power,-power);
      }
      System.out.println("Target: " + angle);
      System.out.println("Current Angle: " +currentAngle);
      System.out.println("Power: " + power);
    }
    else if (angle < 0) {
      power *= -1;
      if (power < -.15) {
        RobotContainer.drive.drivePercentOutput(.15, -.15);
      }
      else RobotContainer.drive.drivePercentOutput(power,-power);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((timer.get() > .5) && (Math.abs(RobotContainer.drive.getLeftMotorOutput()) <= .08));
  }
}
