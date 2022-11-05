package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
	public static double collection=0;
	public static double ground=0;
    public double nextLevel;


	public Lift(HardwareMap map, Telemetry telemetry){
		this.telemetry = telemetry;

		liftMotorLeft =map.get(DcMotor.class,"liftLeft"); /*the link between the code and the physical motor*/
		liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotorRight =map.get(DcMotor.class,"liftRight"); /*the link between the code and the physical motor*/
		liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


	}

	public void test (){

	}

	public void runToPosition(double position, double speed) {
		// receives speed and positional target and moves motor to target
		int newLiftTarget = (int) (position*COUNTS_PER_INCH);
		liftMotorLeft.setTargetPosition(newLiftTarget);
		liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotorLeft.setPower(speed);

		liftMotorRight.setTargetPosition(newLiftTarget);
		liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotorRight.setPower(speed);

		while (liftMotorLeft.isBusy() || (liftMotorRight.isBusy())) {
			telemetry.addData("Lift Motor Position: ", getCurrentPosition());
			telemetry.update();
		}

		liftMotorLeft.setPower(0);
		liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		liftMotorRight.setPower(0);
		liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	 }

	public double getCurrentPosition() {
		return liftMotorLeft.getCurrentPosition();
	}

	public void setNextLevel(double level){
		this.nextLevel=level;
	}
}
