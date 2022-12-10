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
	Controller controller;


	@Override
	public void runOpMode() throws InterruptedException{
		GamepadEx gamepad1ex = new GamepadEx(gamepad1);
		controller = new Controller(gamepad1ex);

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
			if (controller.flipMiddleButton.wasJustPressed()){
				robot.flip.holdPosition();
			}

			if (controller.flipDownButton.wasJustPressed()){
				robot.flip.resetPosition();
			}

			if (controller.liftUpButton.wasJustPressed()){
				robot.lift.moveUpOneLevel();
			}

			if(controller.liftDownButton.wasJustPressed()){
				robot.lift.moveDownOneLevel();
			}

			if(controller.liftMoveButton.wasJustPressed()) {
				robot.lift.moveLift();
			}
			if(controller.swingFrontButton.wasJustPressed()) {
				robot.arm.setArmPositionUp();
			}
			if (controller.swingBackButton.wasJustPressed ()) {
				robot.arm.setArmPositionDown();
			}
			if (controller.clawButton.wasJustPressed()) {
				robot.claw.OpenPosition();
			}
			if (controller.EmergancyClose.wasJustPressed()){
				robot.claw.ClosePosition();

			}




			controller.readButtons();
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


