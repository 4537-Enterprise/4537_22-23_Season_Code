package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift{
	//identifies lift motors and controls motor movement

	//highTerminal
	//medTerminal
	//lowTerminal
	//goundTerminal
	//active
	//collection
	//ground
	//ground and collection are interchangeable with the same value

	Telemetry telemetry;

	public DcMotor liftMotorLeft;
	public DcMotor liftMotorRight;

	//these constants define how many rotations are in an inch
	static final double COUNTS_PER_MOTOR_REV = 28;
	static final double DRIVE_GEAR_REDUCTION = 1.0;
	static final double WHEEL_DIAMETER_INCHES = 2.0;
	static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


	//these constants define the different heights the lift will be
	public static double highTerminal=100;
	public static double medTerminal=75;
	public static double lowTerminal=50;
	public static double groundTerminal=25;
	public static double active=15;
	public static double ground=0;
    public double nextLevel;

	public String currPosition = "active";

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

		liftMotorLeft = map.get(DcMotor.class,"liftLeft"); /*the link between the code and the physical motor*/
		liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		liftMotorRight = map.get(DcMotor.class,"liftRight"); /*the link between the code and the physical motor*/
		liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		liftPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
		liftPID = new PIDEx(liftPIDCoefficients);
	}

	public void setPower(double power){
		liftMotorLeft.setPower(power);
		liftMotorRight.setPower(power);
	}

	public void stop(){
		liftMotorLeft.setPower(0);
		liftMotorRight.setPower(0);
	}

	public double getCurrentPosition() {
		double leftValue = liftMotorLeft.getCurrentPosition();
		double rightValue = liftMotorRight.getCurrentPosition();
		double avgValue = (leftValue+rightValue)/2;
		return avgValue;
	}

	public void setNextLevel(double level){
		this.nextLevel=level;
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
				liftPID.calculate(ground, getCurrentPosition()),
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
