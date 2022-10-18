package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift{
	Telemetry telemetry;

	private DcMotor liftMotor;

	static final double COUNTS_PER_MOTOR_REV = 28;
	static final double DRIVE_GEAR_REDUCTION = 1.0;
	static final double WHEEL_DIAMETER_INCHES = 2.0;
	static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

	public Lift(HardwareMap map, Telemetry telemetry){
		this.telemetry = telemetry;

		liftMotor=map.get(DcMotor.class,"liftMotor"); /*the link between the code and the physical motor*/
		liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void runToPosition(double position, double speed) {
		int newLiftTarget = (int) (position*COUNTS_PER_INCH);
		liftMotor.setTargetPosition(newLiftTarget);

		liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor.setPower(speed);

		while (liftMotor.isBusy()) {
			telemetry.addData("Lift Motor Positon: ", getCurrentPosition());
			telemetry.update();
		}

		liftMotor.setPower(0);
		liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public double getCurrentPosition() {
		return liftMotor.getCurrentPosition();
	}
}
