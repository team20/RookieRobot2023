/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 // we also dont know what to do with this subsystem because we dont really need it but we dont want to get rid of it so deal with it (for now)

 package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakePneumaticSubsystem extends SubsystemBase {

    Compressor compressor;
    DoubleSolenoid intake; 

    public IntakePneumaticSubsystem() {
        compressor = new Compressor(1, PneumaticsModuleType.REVPH);

        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5); // TODO: check ports, currently guessing
    }
    
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}