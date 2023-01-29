/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {

    Compressor compressor;
    DoubleSolenoid intake; 

    public PneumaticSubsystem(int solPort1, int solPort2) {
        Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solPort1, solPort2); // TODO: check ports, currently guessing
    }
    // TODO: check this idk
    private PneumaticSubsystem clawPneumatic(2, 3);
    private PneumaticSubsystem pivotPneumatic(4, 5);
    private PneumaticSubsystem pressurePneumatic(6, 7);
    private PneumaticSubsystem smthPneumatic(8, 9);

    public static final class IntakeMethods {
		// for intake pneumatics but we don't have intake pneumatics so something's wrong here but we don't really care
        public void stop() {
            compressor.disable();
        }
    
        public void lowerIntake() {
            intake.set(DoubleSolenoid.Value.kReverse);
        }
    
        public void raiseIntake() {
            intake.set(DoubleSolenoid.Value.kForward);
        }
    
        public void toggleIntake() {
            intake.toggle();
        }
	}
    
    public static final class ClawMethods {
		// we actually have something for this so yay
        public void openClaw() {
            claw.set(DoubleSolenoid.Value.kForward);
        }
    
        public void closeClaw() {
            claw.set(DoubleSolenoid.Value.kReverse);
        }   
	}
}