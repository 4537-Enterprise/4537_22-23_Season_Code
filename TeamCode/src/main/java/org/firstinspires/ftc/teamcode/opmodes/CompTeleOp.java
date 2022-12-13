package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.Controls.Controller;

@TeleOp(name = "CompTeleOp")
public class CompTeleOp extends LinearOpMode{
	CompRobot robot;

	double speedOverride = 1;

	
	//TODO: Tell engineers to replace expansion hub on top,

	//Pose2d poseEstimate;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();
	Controller payloadController;
	
	@Override
	public void runOpMode() throws InterruptedException{
		GamepadEx payloadcontrollerEx = new GamepadEx(gamepad2);
		//GamepadEx driverController = new GamepadEx(gamepad1);
		payloadController = new Controller (payloadcontrollerEx);
		robot = new CompRobot(hardwareMap, telemetry);

		//Move lift to active position
		robot.lift.setNextLevel(robot.lift.active);
		robot.lift.moveLift();

		double ClawPosition = robot.claw.getPosition();
		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()){
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while (opModeIsActive()){

			if (gamepad1.left_trigger > 0.5) {
				speedOverride = 0.25;
			}

			else if (gamepad1.right_trigger > 0.5) {
				speedOverride = 0.5;
			}
			else {
				speedOverride = 1;
			}


			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y * speedOverride,
							-gamepad1.left_stick_x * speedOverride,
							-gamepad1.right_stick_x * speedOverride
					)
			);
			robot.drive.update();
			//	robot.drive.update();
			if (payloadController.flipMiddleButton.wasJustPressed()){
				robot.flip.holdPosition();
			}

			if (payloadController.flipDownButton.wasJustPressed()){
				robot.flip.resetPosition();
			}

			if (payloadController.liftUpButton.wasJustPressed()){
				robot.lift.moveUpOneLevel();
			}

			if(payloadController.liftDownButton.wasJustPressed()){
				robot.lift.moveDownOneLevel();
			}

			if(payloadController.liftMoveButton.wasJustPressed()) {
				robot.lift.moveLift();
			}
			if(payloadController.swingFrontButton.wasJustPressed()) {
				robot.arm.setArmPositionUp();
			}
			if (payloadController.swingBackButton.wasJustPressed ()) {
				robot.arm.setArmPositionDown();
			}
			if (payloadController.clawButton.wasJustPressed()) {
				robot.claw.OpenPosition();
			}
			if (payloadController.EmergancyClose.wasJustPressed()){
				robot.claw.ClosePosition();

			}

			payloadController.readButtons();
			//poseEstimate = robot.drive.getPoseEstimate();
			telemetry.addData("ClawSensor", robot.claw.ClawSensor.isPressed());
			telemetry.addData("liftNextPos", robot.lift.nextPosition);
			telemetry.addData("liftCurrentPos", robot.lift.currPosition);
			telemetry.addData ("is arm up", robot.arm.isArmUp);
			telemetry.addData ("Arm Position", robot.arm.CurrPosition);
			//telemetry.addData("x", poseEstimate.getX());
			//telemetry.addData("y", poseEstimate.getY());
			//telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
		}


	}
}


