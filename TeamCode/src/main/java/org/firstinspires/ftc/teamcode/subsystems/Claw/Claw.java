package org.firstinspires.ftc.teamcode.subsystems.Claw;

import android.widget.Switch;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw{
    public TouchSensor ClawSensor;
    private Servo Claw;
    /*the link between the code and the physical motor*/

    public static double OPEN_POSITION = 1;
    public static double CLOSE_POSITION = 0;

    public Claw(HardwareMap map){
        ClawSensor = map.get(TouchSensor.class, "clawSensor");
        Claw = map.get(Servo.class, "claw"); /*the link between the code and    private Servo Claw;
    /*the link between the code and the physical motor*/
    }


    public double getPosition(){
        return Claw.getPosition();
    }

    public void setPosition(double position){
        Claw.setPosition(position);
    }

    public void OpenPosition(){
        Claw.setPosition(OPEN_POSITION);
    }

    public void ClosePosition(){
        if (ClawSensor.isPressed()){
            Claw.setPosition(CLOSE_POSITION);

        }
    }
}