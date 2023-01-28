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

public class HatchCollectorSubsystem extends SubsystemBase {

    Compressor compressor;
    DoubleSolenoid claw; //first number is forward channel, second number is reverse channel

    /**
     * Creates a new ExampleSubsystem.
     */
    public HatchCollectorSubsystem() {
        compressor = new Compressor();
        compressor.setClosedLoopControl(true);

        claw = new DoubleSolenoid(2, 3); //first number is forward channel, second number is reverse channel
    }

    public void openClaw() {
        claw.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
        claw.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
