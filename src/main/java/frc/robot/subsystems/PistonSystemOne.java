// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//look up a double 
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PistonMap;
/**import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonMap.Button.*;
import frc.robot.util.controllers.GameController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller.Button;
*/

public class PistonSystemOne extends SubsystemBase {
  /** Creates a new FrictionPad. */
  
  private DoubleSolenoid piston;
  public static PistonSystemOne instance;
  
  public static PistonSystemOne getInstance(){
    if(instance == null){
      instance = new PistonSystemOne();   
    }
    return instance;
  }

  /**
   * attemts to create singleton
   * Cheack how to make it static
   */
  private PistonSystemOne() {   
        piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,PistonMap.FOWARD_CHANNEL, PistonMap.REVERSE_CHANNEL);   
  }

  public void shootPiston(){
    piston.toggle(); // sets to other   
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
