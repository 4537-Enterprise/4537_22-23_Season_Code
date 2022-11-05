package org.firstinspires.ftc.teamcode.subsystems.ArmSwing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSwing{

	public DcMotor Arm;
//one rotation = 288 ticks
	public static double armUpPosition = 80;
	public static double armUpMax= 100;
	public static double armDownPosition = -80;
	public static double armDownMax= -100;
    public static double speed = 1;
	public static boolean isArmUp = false;
	public ArmSwing(HardwareMap map){

		Arm = map.get(DcMotor.class, "arm"); /*the link between the code and the physical motor*/
		Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}


	public void setArmPositionUp(){
		if (isArmUp== false){


			int newLiftTarget = (int) (armUpPosition);
			Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

			Arm.setDirection(DcMotorSimple.Direction.FORWARD);

			Arm.setTargetPosition(newLiftTarget);

			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			while (Arm.getCurrentPosition()<armUpMax){

				Arm.setPower(speed);
			}
			isArmUp = true;
		}
	}
	public void setArmPositionDown(){

		if (isArmUp == true){

			int newLiftTarget = (int) (armDownPosition);
			Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

			Arm.setDirection(DcMotorSimple.Direction.FORWARD);

			Arm.setTargetPosition(newLiftTarget);

			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			while (Arm.getCurrentPosition()>armDownMax){

				Arm.setPower(-speed);
			}
			isArmUp = false;
		}
	}


}