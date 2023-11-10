// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class BrakePneumaticsSubsystem extends SubsystemBase {
    private DoubleSolenoid m_brake;
    private Value m_state;
    private static BrakePneumaticsSubsystem m_subsystem;
    
    public BrakePneumaticsSubsystem() {
        if (m_subsystem != null) {
            try {
            throw new Exception("Brake subsystem already initalized!");
            } catch (Exception e) {
            e.printStackTrace();
            }
        }else{
            m_subsystem = this;
        }
        m_brake = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kBrakeFwdPort, PneumaticConstants.kBrakeRevPort);
        m_state = m_brake.get();
    }

    public static BrakePneumaticsSubsystem get(){
        return m_subsystem;
    }

    public void armBrake() {
        m_brake.set(DoubleSolenoid.Value.kReverse);
    }

    public void armInMovement() {
        m_brake.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Gets the current state of the brake pneumatic
     * @return the curent double solenoid value
     * @see Value
     */
    public Value getValue(){
        return m_state;
    }
}   
