package org.firstinspires.ftc.teamcode.subsystems.Flip;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Flipper{
	private Servo Flip;

	public static double FLIP_POSITION = 1;
	public static double RESET_POSITION = 0.3;
	public static double HOLD_POSITION = 0;

	public Flipper(HardwareMap map){

		Flip=map.get(Servo.class,"flip"); /*the link between the code and the physical motor*/

	}

	public void setPosition(double position) {
		Flip.setPosition(position);
	}

	public void flipPosition()  { Flip.setPosition(FLIP_POSITION); }

	public void resetPosition() {
		Flip.setPosition(RESET_POSITION);
	}

	public void holdPosition()  { Flip.setPosition(HOLD_POSITION); }



}

