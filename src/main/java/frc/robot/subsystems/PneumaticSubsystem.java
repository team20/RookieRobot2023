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
        m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solPort1, solPort2); // TODO: check ports, currently guessing
    }
}