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
import org.firstinspires.ftc.teamcode.subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.Controls.Controller;

@TeleOp(name = "CompTeleOp")
public class CompTeleOp extends LinearOpMode{
	//dpad sets the pre determined heights and the right trigger executes the program
	CompRobot robot;

	double speedOverride = 1;
	//TODO : Implement Controller methods for lift,
	//GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
	//GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

	Pose2d poseEstimate;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();
	Controller controller;


	@Override
	public void runOpMode() throws InterruptedException{
		GamepadEx gamepad1ex = new GamepadEx(gamepad1);
		//GamepadEx gamepad2ex = new GamepadEx(gamepad2);
		controller = new Controller(gamepad1ex);


		robot = new CompRobot(hardwareMap, telemetry);

		double liftPos = robot.lift.getCurrentPosition();

		double ClawPosition = robot.claw.getPosition();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()) {
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);


		}

		while(opModeIsActive()){
//			if (gamepad1.left_trigger > 0.5){
//				speedOverride = 0.25;
//			} else if (gamepad1.right_trigger > 0.5){
//				speedOverride = 0.5;
//			} else{
//				speedOverride = 1;
//			}
//
//			robot.drive.setWeightedDrivePower(
//					new Pose2d(
//							-gamepad1.left_stick_y * speedOverride,
//							-gamepad1.left_stick_x * speedOverride,
//							-gamepad1.right_stick_x * speedOverride
//					)
		//	);
			robot.drive.update();
			if (controller.flipMiddleButton.wasJustPressed()){
				robot.flip.holdPosition();
			}

			if (controller.flipDownButton.wasJustPressed()){
				robot.flip.resetPosition();
			}
//			if (gamepad1.x){
//				robot.flip.flipPosition();
//			}
//
//			if (gamepad1.dpad_right){
//				robot.arm.setArmPositionUp();
//			}
//			if (gamepad1.dpad_left){
//
//				robot.arm.setArmPositionDown();
//			}

			if (controller.liftMoveButton.wasJustPressed()){

				if (liftPos == robot.lift.collection){
					robot.lift.setNextLevel(robot.lift.active);
				}

				if (liftPos == robot.lift.active){
					robot.lift.setNextLevel(robot.lift.groundTerminal);
				}

				if (liftPos == robot.lift.groundTerminal){
					robot.lift.setNextLevel(robot.lift.lowTerminal);
				}

				if (liftPos == robot.lift.lowTerminal){
					robot.lift.setNextLevel(robot.lift.medTerminal);
				}

				if (liftPos == robot.lift.medTerminal){
					robot.lift.setNextLevel(robot.lift.highTerminal);
				}


			}
			if (controller.liftDownButton.wasJustPressed()){
				if (liftPos == robot.lift.highTerminal){
					robot.lift.setNextLevel(robot.lift.medTerminal);
				}

				if (liftPos == robot.lift.medTerminal){
					robot.lift.setNextLevel(robot.lift.lowTerminal);
				}

				if (liftPos == robot.lift.lowTerminal){
					robot.lift.setNextLevel(robot.lift.groundTerminal);
				}

				if (liftPos == robot.lift.groundTerminal){
					robot.lift.setNextLevel(robot.lift.active);
				}

				if (liftPos == robot.lift.active){
					robot.lift.setNextLevel(robot.lift.collection);
				}
			}


			/*if gamepad2.right_trigger {

			}*/


			poseEstimate = robot.drive.getPoseEstimate();
			telemetry.addData("liftPos", robot.lift.nextLevel);
			telemetry.addData("liftCurrentPos", robot.lift.getCurrentPosition());
			telemetry.addData("x", poseEstimate.getX());
			telemetry.addData("y", poseEstimate.getY());
			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
		}

	}
}
