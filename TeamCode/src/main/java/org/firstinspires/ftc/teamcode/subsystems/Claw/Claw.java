package org.firstinspires.ftc.teamcode.subsystems.Claw;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    private Servo Claw;
    /*the link between the code and the physical motor*/

    public static double OPEN_POSITION = 1;
    public static double CLOSE_POSITION = 0;

    public Claw(HardwareMap map) {

        Claw= map.get(Servo.class, "claw"); /*the link between the code and the physical servo*/

    }

    public void setPosition(double position) {
        Claw.setPosition(position);
    }

    public void OpenPosition() {
        Claw.setPosition(OPEN_POSITION);
    }

    public void ClosePosition() {
        Claw.setPosition(CLOSE_POSITION);
    }
}