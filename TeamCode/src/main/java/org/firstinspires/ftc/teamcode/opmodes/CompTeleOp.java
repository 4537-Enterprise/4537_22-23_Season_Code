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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Controls.DriverController;


@TeleOp(name = "CompTeleOp")
public class CompTeleOp extends LinearOpMode{
	CompRobot robot;
	double directionControl = 1; //One Forward = 1 and -1 = backwards
	double speedOverride = 1;
	int tempLiftLevelInt = 0;
	String tempLiftLevelString = "lollllllllll";

	String directionControlstring = "forward";

	
	//TODO: Tell engineers to replace expansion hub on top,

	//Pose2d poseEstimate;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();
	Controller payloadController;
	DriverController driverController;
	
	@Override
	public void runOpMode() throws InterruptedException{
		GamepadEx payloadcontrollerEx = new GamepadEx(gamepad2);
		GamepadEx driverControllerEx = new GamepadEx(gamepad1);
		payloadController = new Controller (payloadcontrollerEx);
		driverController = new DriverController (driverControllerEx);
		robot = new CompRobot(hardwareMap, telemetry);

		//Move lift to active position
		robot.lift.setNextLevel(robot.lift.active);
		robot.lift.moveLift();
		//Move Arm to specified location
		robot.arm.setArmPositionInitilize();

		double ClawPosition = robot.claw.getPosition();
		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()){
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		//TODO why isnt this working?
		while (opModeIsActive()){

			if (driverController.halfSpeedButton.wasJustPressed()){ //Y button
				speedOverride = 0.5;
				telemetry.addData("hello","halfspeed");
			}
			if (driverController.threeFourthSpeedButton.wasJustPressed()){  //B button
				speedOverride = 0.75;
				telemetry.addData("hello","Fasterspeed");
			}
			if (driverController.fullSpeedButton.wasJustPressed()){ //A Button
				speedOverride = 1;
				telemetry.addData("hello","Fullspeed");
			}

			if (driverController.switchDirectionsButton.wasJustPressed()){
				directionControl *= -1;
				if (directionControl == 1){
					directionControlstring =  "forward";
				} else if (directionControl == -1) {
					directionControlstring = "Backward";
				} else {
					directionControlstring = "good luck";
				}
			}


			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y * speedOverride * directionControl,
							-gamepad1.left_stick_x * speedOverride * directionControl,
							-gamepad1.right_stick_x * speedOverride *directionControl
					)
			);
			robot.drive.update();
			//	robot.drive.update();
			//if (payloadController.flipMiddleButton.wasJustPressed()){
			//	robot.flip.holdPosition();
		//	}

			if (payloadController.flipDownButton.wasJustPressed()){
				robot.flip.resetPosition();
			}

			if (payloadController.liftUpButton.wasJustPressed()){
				robot.lift.moveUpOneLevel();
			}

			if (payloadController.liftDownButton.wasJustPressed()){
				robot.lift.moveDownOneLevel();
			}

			if (payloadController.liftMoveTrigger.wasJustPressed()){
				robot.lift.moveLift();
			}
			if (payloadController.swingFrontTrigger.wasJustPressed()&& robot.lift.currPosition == "highTerminal"){
				robot.arm.setArmPositionUp();
			}
			if (payloadController.swingBackButton.wasJustPressed() && robot.lift.currPosition == "highTerminal"){
				robot.arm.setArmPositionDown();
			}
			if (payloadController.clawButton.wasJustPressed()){
				robot.claw.OpenPosition();
			}
			if (payloadController.EmergancyClose.wasJustPressed()){
				robot.claw.ClosePosition();
			}
			if (payloadController.armPickUpButton.wasJustPressed()){
				robot.arm.setArmPickUp();
			}
			if (gamepad2.left_stick_y >= 0.5){
				robot.arm.moveArmDownManual();
			}
			if (gamepad2.left_stick_y <= -0.5){
				robot.arm.moveArmUpManual();
			}
			if (gamepad2.right_stick_y >= 0.5){
				robot.lift.moveLiftDownManual();
			}
			if (gamepad2.right_stick_y <= -0.5){
				robot.lift.moveLiftUpManual();
			}
			if (payloadController.lowestLevelButton.wasJustPressed()) {
				tempLiftLevelString = robot.lift.currPosition;
				switch (tempLiftLevelString){
					case "active":
						tempLiftLevelInt = robot.lift.active;
						break;
					case "ground":
						tempLiftLevelInt = robot.lift.ground;
						break;
					case "groundTerminal":
						tempLiftLevelInt = robot.lift.groundTerminal;
						break;
					case "lowTerminal":
						tempLiftLevelInt = robot.lift.lowTerminal;
						break;
					case "medTerminal":
						tempLiftLevelInt = robot.lift.medTerminal;
						break;
					case "highTerminal":
						tempLiftLevelInt = robot.lift.highTerminal;
						break;
				}

				robot.lift.setNextLevel(robot.lift.active);
				robot.lift.nextPosition = "active";
				robot.lift.moveLift();
				robot.lift.setNextLevel(tempLiftLevelInt);
				robot.lift.nextPosition = tempLiftLevelString;
			}

			driverController.readButtons();
			payloadController.readButtons();
			//poseEstimate = robot.drive.getPoseEstimate();
			//telemetry.addData("ClawSensor", robot.claw.ClawSensor.isPressed());
			telemetry.addData("liftNextPos", robot.lift.nextPosition);
			telemetry.addData("liftCurrentPos", robot.lift.currPosition);
			telemetry.addData ("is arm up", robot.arm.isArmUp);
			telemetry.addData ("Arm Position", robot.arm.Arm.getCurrentPosition());
			telemetry.addData ("Speed", speedOverride);
			telemetry.addData ("Direction", directionControlstring);
			telemetry.addData ("Some lift BS", robot.lift.nextLevel);
			//telemetry.addData("x", poseEstimate.getX());
			//telemetry.addData("y", poseEstimate.getY());
			//telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
			FtcDashboard.getInstance().getTelemetry().addData("Arm Position", robot.arm.Arm.getCurrentPosition());
		}
	}
}


