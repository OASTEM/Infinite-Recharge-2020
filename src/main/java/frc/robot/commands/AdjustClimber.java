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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NavX;

public class AdjustClimber extends CommandBase {
  /**
   * Creates a new AdjustClimber.
   */
  private Climber climber;
  private NavX navX;
  private Timer timer = new Timer();
  public AdjustClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = RobotContainer.climber;
    this.navX = RobotContainer.navX;
    addRequirements(climber);
    addRequirements(navX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(navX.getAccelY() < -.1 && climber.getLeftPosition() > 50) {
      climber.runLeftMotor(-0.2);
    }
    else if (navX.getAccelY() > .1 && climber.getRightPosition() < -50) {
      climber.runRightMotor(0.2);
    }
    System.out.println("Y: " + RobotContainer.navX.getAccelY());
    
  }


// left (/) >> left down / right up
// right (\) >> right down / left up 


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runLeftMotor(0.0);
    climber.runRightMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 2 && (Math.abs(navX.getAccelY()) < .1));
  }
}
