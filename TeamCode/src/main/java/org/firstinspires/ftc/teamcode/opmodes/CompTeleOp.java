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

import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@TeleOp(name = "CompTeleOp")
public class CompTeleOp extends LinearOpMode{
	//dpad sets the pre determined heights and the right trigger executes the program
	CompRobot robot;

	double speedOverride = 1;

	GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
	GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
	ButtonReader dpadUpReader = new ButtonReader(
			gamepadEx2, GamepadKeys.Button.DPAD_UP
	);
	TriggerReader rightTriggerReader = new TriggerReader(
			gamepadEx2, GamepadKeys.Trigger.RIGHT_TRIGGER
	);

	Pose2d poseEstimate;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	@Override
	public void runOpMode() throws InterruptedException{

		robot = new CompRobot(hardwareMap, telemetry);

		double liftPos = robot.lift.getCurrentPosition();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()) {
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while(opModeIsActive()){
			if (gamepad1.left_trigger > 0.5){
				speedOverride = 0.25;
			} else if (gamepad1.right_trigger > 0.5){
				speedOverride = 0.5;
			} else{
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
			if (gamepad1.b){
				robot.flip.holdPosition();
			}

			if (gamepad1.y){
				robot.flip.resetPosition();
			}
			if (gamepad1.x){
				robot.flip.flipPosition();
			}

			if (gamepad1.dpad_right){
				robot.arm.setArmPositionUp();
			}
			if (gamepad1.dpad_left){

				robot.arm.setArmPositionDown();
			}

			if (gamepad2.dpad_up){

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
			if (gamepad2.dpad_down){
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

			if (rightTriggerReader.wasJustPressed()){

			}

			poseEstimate = robot.drive.getPoseEstimate();
			telemetry.addData("x", poseEstimate.getX());
			telemetry.addData("y", poseEstimate.getY());
			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
		}

	}
}
