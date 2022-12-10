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
	public static double armUpPosition = 5;
	public static double armUpMax= 42;
	public static double armDownPosition = -5;
	public static double armDownMax= -42;
    public static double speed = 1;
	//TODO delete later
	public static double CurrPosition;
	public boolean isArmUp = false;
	public ArmSwing(HardwareMap map){
		Arm = map.get(DcMotor.class, "arm"); /*the link between the code and the physical motor*/
		Arm.setDirection(DcMotorSimple.Direction.FORWARD);
		Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Arm.setPower(speed);
		this.CurrPosition = 0;
	}


	public void setArmPositionUp(){
		this.CurrPosition = this.CurrPosition + 5;
			int newLiftTarget = (int) (armUpPosition);
		    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Arm.setTargetPosition((int)this.CurrPosition);
			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			isArmUp = true;

	}
	public void setArmPositionDown(){
		//TODO: figure out why the backwards motion does n  ot have enough power.
		this.CurrPosition = this.CurrPosition - 5;
			int newLiftTarget = ((int)this.CurrPosition);
			Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Arm.setTargetPosition(newLiftTarget);
			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			isArmUp = false;
	}


}