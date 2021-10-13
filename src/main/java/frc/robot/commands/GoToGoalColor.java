/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GoToGoalColor extends CommandBase {
  /**
   * Creates a new GoToGoalColor.
   */
  String goalColor;

  public GoToGoalColor(String goalColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalColor = goalColor;
    addRequirements(RobotContainer.cp_Man);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Goal Color: " + goalColor);
    if(goalColor.length() > 0) {
      RobotContainer.cp_Man.run(0.08);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(RobotContainer.cp_Man.getColor());
    System.out.println((RobotContainer.cp_Man.getColor().equals(goalColor))); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cp_Man.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalColor.length() == 0 || (RobotContainer.cp_Man.getColor().equals(goalColor.substring(0,1)));
  }
}
