package org.firstinspires.ftc.teamcode.subsystems.ArmSwing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSwing{

	public DcMotor Arm;
	// TODO add telemetry for the arm to measure the max of the robot
//one rotation = 288 ticks
	public static double armUpPosition = 5;
	//true up max value is 225
	public static double armUpMax= 370;
	public static double armInit = 45;
	public static double armDownPosition = -5;
	//true down max value is -1000
	public static double armDownMax= -800;
    public static double speed = 1.0;
	public static double armPickUpPosition = 45;

	PIDEx armPID;
	PIDCoefficientsEx armPIDCoefficients;
	public static double kP = 0.5;
	public static double kI = 0.0;
	public static double kD = 0.0;
	double integralSumMax = 1 / kI;
	double stabilityThreshold = 0.0;
	double lowPassGain = 0.0;

	//TODO delete later
	public static double CurrPosition;
	public boolean isArmUp = false;
	public ArmSwing(HardwareMap map){
		Arm = map.get(DcMotor.class, "arm"); /*the link between the code and the physical motor*/
		Arm.setDirection(DcMotorSimple.Direction.FORWARD);
		Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Arm.setPower(speed);
		this.CurrPosition = Arm.getCurrentPosition();


		armPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
		armPID = new PIDEx(armPIDCoefficients);
	}

	public void setPower(double power){
		Arm.setPower(power);
	}

	public void setArmPositionUp(){
		this.CurrPosition = this.CurrPosition;
			//int newLiftTarget = (int) (armUpPosition);
		    //Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Arm.setTargetPosition((int)armUpMax);
			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			isArmUp = true;

	}
	public void setArmPositionDown(){
		//TODO: figure out why the backwards motion does n  ot have enough power.
		this.CurrPosition = this.CurrPosition;
			int newLiftTarget = ((int)armDownMax);
			//Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Arm.setTargetPosition(newLiftTarget);
			Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			isArmUp = false;
	}
	public void setArmPickUp (){
		this.CurrPosition = this.CurrPosition;
		int newLiftTarget = ((int)armPickUpPosition);
		Arm.setTargetPosition(newLiftTarget);
		Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
	public void setArmPositionInitilize(){
		this.CurrPosition = this.CurrPosition;
		int newLiftTarget = ((int)armInit);
		Arm.setTargetPosition(newLiftTarget);
		Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void moveArmUpManual(){
		int newTarget = Arm.getCurrentPosition() + 20;
		Arm.setTargetPosition(newTarget);
		Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void moveArmDownManual(){
		int newTarget = Arm.getCurrentPosition() - 20;
		Arm.setTargetPosition(newTarget);
		Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void updateArmPID() {
		armPIDCoefficients.Kp = kP;
		armPIDCoefficients.Ki = kI;
		armPIDCoefficients.Kd = kD;
		armPIDCoefficients.maximumIntegralSum = integralSumMax;
		armPIDCoefficients.stabilityThreshold = stabilityThreshold;
		armPIDCoefficients.lowPassGain = lowPassGain;
	}


	public void updateIdle(){
		updateArmPID();
		double power = Range.clip(
				armPID.calculate(armInit, Arm.getCurrentPosition()),
				-1,
				1
		);

		if (power >= -0.1){
			setPower(power);
		} else{
			setPower(power * 0.1);
		}

	}
}