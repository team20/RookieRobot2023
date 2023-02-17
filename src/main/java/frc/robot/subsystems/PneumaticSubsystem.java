/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {

    Compressor compressor;
    DoubleSolenoid intake;
    
    public static Solenoid phosLipSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

    //phos.set​(true);


    // public PneumaticSubsystem(int solPort1, int solPort2) {
    //     Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    //     DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solPort1, solPort2); // TODO: check ports, currently guessing
    // }
    // // TODO: check this idk
    // private PneumaticSubsystem clawPneumatic = new PneumaticSubsystem(2, 3);
    // private PneumaticSubsystem pivotPneumatic = new PneumaticSubsystem(2, 3);
    // private PneumaticSubsystem pressurePneumatic = new PneumaticSubsystem(2, 3);
    // private PneumaticSubsystem smthPneumatic = new PneumaticSubsystem(2, 3);

    // public final class IntakeMethods {
	// 	// for intake pneumatics but we don't have intake pneumatics so something's wrong here but we don't really care
    //     public void stop() {
    //         compressor.disable();
    //     }
    
    //     public void lowerIntake() {
    //         intake.set(DoubleSolenoid.Value.kReverse);
    //     }
    
    //     public void raiseIntake() {
    //         intake.set(DoubleSolenoid.Value.kForward);
    //     }
    
    //     public void toggleIntake() {
    //         intake.toggle();
    //     }
	// }
    
    // public final class ClawMethods {
	// 	// we actually have something for this so yay
    //     // public void openClaw() {
    //     //     clawPneumatic.set(DoubleSolenoid.Value.kForward);

    //     // }
    
    //     // public void closeClaw() {
    //     //     clawPneumatic.set(DoubleSolenoid.Value.kReverse);
    //     // }   
	// }
}


