// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class ClawPneumaticsSubsystem extends SubsystemBase {
    private DoubleSolenoid m_claw;
    private Value m_state;
    private static ClawPneumaticsSubsystem m_subsystem;

    public ClawPneumaticsSubsystem() {
        if (m_subsystem != null) {
            try {
            throw new Exception("Claw subsystem already initalized!");
            } catch (Exception e) {
            e.printStackTrace();
            }
        }else{
            m_subsystem = this;
        }
        m_claw = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kClawFwdPort, PneumaticConstants.kClawRevPort);
        m_state = m_claw.get();
    }

    public static ClawPneumaticsSubsystem get(){
        return m_subsystem;
    }

    public void openClaw() {
        m_claw.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
        m_claw.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleClaw() {
        m_claw.toggle();
    }

    /**
     * Gets the current state of the claw pneumatic
     * @return the curent double solenoid value
     * @see Value
     */
    public Value getValue(){
        return m_state;
    }
}