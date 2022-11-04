package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@TeleOp(name = "CompTeleOp")
public class CompTeleOp extends LinearOpMode{

	CompRobot robot;

	double speedOverride = 1;

	Pose2d poseEstimate;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	@Override
	public void runOpMode() throws InterruptedException{

		robot = new CompRobot(hardwareMap, telemetry);

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()) {
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while(opModeIsActive()) {
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
			if (gamepad1.b) {
				robot.flip.holdPosition();
			}

			if (gamepad1.y) {
				robot.flip.resetPosition();
			}
			if (gamepad1.x) {
				robot.flip.flipPosition();
			}

			if (gamepad1.dpad_right) {

				robot.arm.setArmPositionUp();
			}
			if (gamepad1.dpad_left) {

				robot.arm.setArmPositionDown();
			}

			if (gamepad1.dpad_up) {
				robot.lift.runToPosition (1000.0,6.0);

			}
			if (gamepad1.dpad_down) {
				robot.lift.runToPosition (-1000.0,6.0);

			}

			poseEstimate = robot.drive.getPoseEstimate();
			telemetry.addData("x", poseEstimate.getX());
			telemetry.addData("y", poseEstimate.getY());
			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
		}

	}
}
