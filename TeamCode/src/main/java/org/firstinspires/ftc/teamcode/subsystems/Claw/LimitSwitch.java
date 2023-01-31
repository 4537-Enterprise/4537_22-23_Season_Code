package org.firstinspires.ftc.teamcode.subsystems.Claw;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LimitSwitch{

	DigitalChannel Limit;
	public DigitalChannel LimitSwitch;

	public boolean limitSwitchOn = true;

	public LimitSwitch(HardwareMap map){
		LimitSwitch = map.get(DigitalChannel.class, "limitSwitch");
	}
}
