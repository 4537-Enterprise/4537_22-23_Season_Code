package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@Autonomous(name = "CompAutoStrafeRight")
public class CompAutoStrafeRight extends LinearOpMode{
	CompRobot robot;

	Pose2d poseEstimate;

	Trajectory traj1;

	enum TrajectoryState {
		TRAJ1,
		IDLE
	}

	TrajectoryState trajectoryState = TrajectoryState.TRAJ1;

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	@Override
	public void runOpMode() throws InterruptedException{

		robot = new CompRobot(hardwareMap, telemetry);

		// Define our start pose
		Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0.0));
		robot.drive.setPoseEstimate(startPose);

		traj1 = robot.drive.trajectoryBuilder(startPose)
				.strafeRight(20)
				.build();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()) {
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		/*Running OpMode Loop*/
		while (opModeIsActive()) {
			robot.drive.update();
			telemetry.update();
			poseEstimate = robot.drive.getPoseEstimate();
			PoseStorage.currentPose = poseEstimate;

			switch (trajectoryState) {
				case TRAJ1:
					robot.drive.followTrajectoryAsync(traj1);
					trajectoryState = TrajectoryState.IDLE;
					break;

				case IDLE:
					if (!robot.drive.isBusy()) {
						requestOpModeStop();
					}
					break;
			}
		}
	}
}
