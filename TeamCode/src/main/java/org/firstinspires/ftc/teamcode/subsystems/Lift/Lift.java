package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift{
	//identifies lift motors and controls motor movement
	Telemetry telemetry;
	public DcMotor liftMotor;

	//these constants define how many rotations are in an inch
	static final double COUNTS_PER_MOTOR_REV = 28;
	static final double DRIVE_GEAR_REDUCTION = 1.0;
	static final double WHEEL_DIAMETER_INCHES = 2.0;
	// Hard-coded for now
	static final int COUNTS_PER_INCH = 420;
//	static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

	//these constants define the different heights the lift will be
	public static final int highTerminal=15;// to the height of 36 inches, high terminal 33, 1/4
	public static final int lowHigh=12;// DO NOT USE IN TELEOP
	public static final int medTerminal=10;//adding 1 //at 27 inches, up three inches
	public static final int Highlow=7;
	public static final int lowTerminal=7; //adding 2//adding 1 //14 inches, up 8 inches
	public static final int lowlow=4; //DO NOT USE IN TELEOP
	//	public static final int groundTerminal=4;
//	public static final int ground=1;
	public static final int active=0;

	public int nextLevel = 0;


	public String currPosition;
	public String nextPosition = currPosition;
	public double liftPower = 0.8;

	PIDEx liftPID;
	PIDCoefficientsEx liftPIDCoefficients;
	public static double kP = 0.5;
	public static double kI = 0.0;
	public static double kD = 0.0;
	double integralSumMax = 1 / kI;
	double stabilityThreshold = 0.0;
	double lowPassGain = 0.0;

	public Lift(HardwareMap map, Telemetry telemetry){
		this.telemetry = telemetry;
		liftMotor = map.get(DcMotor.class, "liftMotor"); /*the link between the code and the physical motor*/
		liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		liftPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
		liftPID = new PIDEx(liftPIDCoefficients);
		this.nextLevel = (int)getCurrentPosition();
	}
	public void moveUpOneLevel() {
		switch(this.nextLevel) {
			case active:
				this.setNextLevel(this.lowTerminal);
				this.nextPosition = "lowTerminal";
				break;
//			case ground:
//				this.setNextLevel(this.groundTerminal);
//				this.nextPosition = "groundTerminal";
//				break;
//			case groundTerminal:
//				this.setNextLevel(this.lowTerminal);
//				this.nextPosition = "lowTerminal";
//				break;
			case lowTerminal:
				this.setNextLevel(this.medTerminal);
				this.nextPosition = "medTerminal";
				break;
			case medTerminal:
				this.setNextLevel(this.highTerminal);
				this.nextPosition = "highTerminal";
				break;
		}
	}

	public void moveDownOneLevel() {
		switch(this.nextLevel) {
			case lowlow:
				this.setNextLevel(active);
				this.nextPosition = "active";

			case active:
				this.setNextLevel(this.active);
				this.nextPosition = "active";
			break;
//			case groundTerminal:
//				this.setNextLevel(this.ground);
//				this.nextPosition = "ground";
//				break;
//			case lowTerminal:
//				this.setNextLevel(this.groundTerminal);
//				this.nextPosition = "groundTerminal";
//				break;
			case medTerminal:
				this.setNextLevel(this.lowTerminal);
				this.nextPosition = "lowTerminal";
				break;
			case highTerminal:
				this.setNextLevel(medTerminal);
				this.nextPosition = "medTerminal";
				break;
		}
	}

	public void setPower(double power){
		liftMotor.setPower(power);
	}

	public void stop(){
		liftMotor.setPower(0);
	}

	public void setNextLevel(int level){
		this.nextLevel=level;
	}

	public double getCurrentPosition() {
		double currentPosition = this.liftMotor.getCurrentPosition();
		this.currPosition = getPositionString((int) currentPosition);
		return currentPosition;
	}

	public String getPositionString(int position) {
		switch(position) {
			case highTerminal:
				return "highTerminal";
			case medTerminal:
				return "medTerminal";
			case lowTerminal:
				return "lowTerminal";

			case active:
				return "active";
			default:
				return "unknown position";
		}
	}

	public int getPositionValue(String position) {
//		if(position == "ground") {
//			return this.ground;
//		}
		if(position == "active") {
			return this.active;
		}
//		else if(position == "groundTerminal") {
//			return this.groundTerminal;
//		}
		else if(position == "lowTerminal") {
			return this.lowTerminal;
		}
		else if(position == "medTerminal") {
			return this.medTerminal;
		}
		else if(position == "highTerminal") {
			return this.highTerminal;
		}
		else {
			return 0;
		}
	}

	public void moveLift() {
		liftMotor.setPower(liftPower);
		this.setPower(liftPower);
		liftMotor.setTargetPosition(this.nextLevel * COUNTS_PER_INCH / 3);
		liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		this.currPosition = this.nextPosition;
	}

	public void moveLiftUpManual(){
		liftMotor.setPower(liftPower);
		this.setPower(liftPower);
		int newTarget = liftMotor.getCurrentPosition() + 120;
		liftMotor.setTargetPosition(newTarget);
		liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
//great work thanks
	public void moveLiftDownManual(){
		this.setPower(liftPower);
		int newTarget = liftMotor.getCurrentPosition() - 120;
		liftMotor.setTargetPosition(newTarget);
		liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void updateLiftPID() {
		liftPIDCoefficients.Kp = kP;
		liftPIDCoefficients.Ki = kI;
		liftPIDCoefficients.Kd = kD;
		liftPIDCoefficients.maximumIntegralSum = integralSumMax;
		liftPIDCoefficients.stabilityThreshold = stabilityThreshold;
		liftPIDCoefficients.lowPassGain = lowPassGain;
	}

	public void updateHoldLevel(){
		updateLiftPID();
		double power = Range.clip(
				liftPID.calculate(nextLevel, getCurrentPosition()),
				-1,
				1
		);

		if(power >= -0.1){
			setPower(power);
		} else {
			setPower(power * 0.1);
		}
	}

	public void updateIdle(){
		updateLiftPID();
		double power = Range.clip(
				liftPID.calculate(active, getCurrentPosition()),
				-1,
				1
		);

		if(power >= -0.1){
			setPower(power);
		} else {
			setPower(power * 0.1);
		}
	}
}
