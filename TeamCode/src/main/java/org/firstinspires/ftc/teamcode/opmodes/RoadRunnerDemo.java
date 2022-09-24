package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@Autonomous(name = "RoadRunnerDemo", group = "test")
public class RoadRunnerDemo extends LinearOpMode{

	CompRobot robot;

	Pose2d poseEstimate;

	Trajectory traj1;
	Trajectory traj2;
	Trajectory traj3;
	Trajectory traj4;

	enum TrajectoryState {
		TRAJ1,
		TRAJ2,
		TRAJ3,
		TRAJ4,
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
				.forward(50)
				.build();

		traj2 = robot.drive.trajectoryBuilder(traj1.end())
				.strafeRight(48)
				.build();

		traj3 = robot.drive.trajectoryBuilder(traj2.end())
				.forward(46)
				.build();

		traj4 = robot.drive.trajectoryBuilder(traj3.end())
				.strafeLeft(48)
				.build();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()) {
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while (opModeIsActive()) {
			robot.drive.update();
			poseEstimate = robot.drive.getPoseEstimate();
			PoseStorage.currentPose = poseEstimate;

			switch (trajectoryState) {
				case TRAJ1:
					robot.drive.followTrajectoryAsync(traj1);
					trajectoryState = TrajectoryState.TRAJ2;
					break;

				case TRAJ2:
					if (!robot.drive.isBusy()) {
						robot.drive.followTrajectoryAsync(traj2);
						trajectoryState = TrajectoryState.TRAJ3;
					}
					break;

				case TRAJ3:
					if (!robot.drive.isBusy()) {
						robot.drive.followTrajectoryAsync(traj3);
						trajectoryState = TrajectoryState.TRAJ4;
					}
					break;

				case TRAJ4:
					if (!robot.drive.isBusy()) {
						robot.drive.turn(360);
						trajectoryState = TrajectoryState.IDLE;
					}
					break;

				case IDLE:
					break;
			}

		}

	}
}
