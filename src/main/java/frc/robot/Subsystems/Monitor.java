/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.AnalogInput;

public class Monitor{
    /// shit fuck oh god
    /// shit fuck monitor temps
    public static AnalogInput Temp0 = new AnalogInput(0);
    public static AnalogInput Temp1 = new AnalogInput(1);
    public static AnalogInput Temp2 = new AnalogInput(2);
    public static AnalogInput Temp3 = new AnalogInput(3);
    

    public double getTemp0()
    {
        Temp0.setAverageBits(4);
        return Temp0.getVoltage();
    }
    public double getTemp1()
    {
        Temp1.setAverageBits(4);
        return Temp1.getVoltage();
    }
    public double getTemp2()
    {
        Temp2.setAverageBits(4);
        return Temp2.getVoltage();
    }
    public double getTemp3()
    {
        Temp3.setAverageBits(4);
        return Temp3.getVoltage();
    }
}

/// takes values from analoug that are hooked to thermocouplers and sends them to shuffleboard - snups
