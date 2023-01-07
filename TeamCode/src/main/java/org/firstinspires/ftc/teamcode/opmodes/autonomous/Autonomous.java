package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "test")
public class Autonomous extends LinearOpMode{

	CompRobot robot;

	Pose2d poseEstimate;

	Trajectory moveToCone;
	Trajectory ditchCone;
	Trajectory moveToSpotOne;
	Trajectory moveToSpotTwo;
	Trajectory forward;
	Trajectory moveToSpotThree;

	enum TrajectoryState{
		MOVE_TO_CONE,
		READ_CONE,
		DITCH_CONE,
		MOVE_TO_SPOT_ONE,
		MOVE_TO_SPOT_TWO,
		MOVE_TO_SPOT_THREE,
		IDLE
	}


	TrajectoryState trajectoryState = TrajectoryState.MOVE_TO_CONE;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0.0));

	@Override
	public void runOpMode() throws InterruptedException{
		robot = new CompRobot(hardwareMap, telemetry);

		// Define our start pose
		// Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0.0));
		robot.drive.setPoseEstimate(startPose);

		moveToCone = robot.drive.trajectoryBuilder(startPose)
				.back(10)
				.build();

		ditchCone = robot.drive.trajectoryBuilder(startPose)
				.back(5)

				.build();

		moveToSpotOne = robot.drive.trajectoryBuilder(moveToCone.end())
				.strafeLeft(22)
				.build();

		moveToSpotTwo = robot.drive.trajectoryBuilder(moveToCone.end())
				.back(30)
				.forward(2)
				.build();


		moveToSpotThree = robot.drive.trajectoryBuilder(moveToCone.end())
				.strafeRight(22)
				.build();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()){
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while (opModeIsActive()){
			robot.drive.update();
			telemetry.update();
			poseEstimate = robot.drive.getPoseEstimate();
			PoseStorage.currentPose = poseEstimate;

			telemetry.addData("RedValue", robot.colorSensor.red);
			telemetry.addData("GreenValue", robot.colorSensor.green);
			telemetry.addData("BlueValue", robot.colorSensor.blue);

			switch (this.trajectoryState){
				case MOVE_TO_CONE:
					robot.drive.followTrajectory(moveToCone);
					this.trajectoryState = TrajectoryState.READ_CONE;
					break;

//				case READ_CONE:
//					robot.colorSensor.getConeColor();
//						if(robot.colorSensor.getConeColor() == "Fuchsia"){
//							robot.drive.followTrajectory(ditchCone);
//							robot.drive.followTrajectory(moveToSpotOne);
//						}
//						if(robot.colorSensor.getConeColor() == "Cyan"){
//						robot.drive.followTrajectory(ditchCone);
//						robot.drive.followTrajectory(moveToSpotTwo);
//				}
//						if(robot.colorSensor.getConeColor() == "Yellow"){
//						robot.drive.followTrajectory(ditchCone);
//						robot.drive.followTrajectory(moveToSpotThree);
//					}
//						break;
//
//
//
//
//			}
//
//			//drive backward x amount up to the cone
//			robot.drive.followTrajectory(moveToCone);
//
//
//			robot.drive.followTrajectory(ditchCone);
//
//
//			// use color sensor to read the color on the cone
//			String coneColor = robot.colorSensor.getConeColor();
//			//if cone is Fuchsia then drive right one tile
//			if(coneColor == "Fuchsia"){
//				telemetry.addLine("PINK");
//				//R = 1
//				//G = .5
//				//B = .8
////				robot.drive.followTrajectory(moveToSpotOne);
//				//then move backward x amount to target high terminal
//				//turn x amount of degrees to face the high terminal
//				// move lift up to high terminal
//				//swing arm above terminal
//				//release claw
//				//end program
//			}
//			// if cone is Cyan
//			else if(coneColor == "Cyan"){
//				telemetry.addLine("Blue");
//				//R: .2
//				//G: .6
//				//B: 1
//				//robot.drive.followTrajectory(moveToSpotTwo);
//				//then drive backward to two-three inches past the high terminal
//				//drive back to high terminal
//				//turn x degrees to high terminal
//				//move lift to high terminal
//				//swing arm to above terminal
//				//release claw
//				//end program
//			}
//			// if cone is Yellow
//			else if(coneColor == "Yellow"){
//				telemetry.addLine("Yellow");
//				//R: .6
//				//G: 1
//				//B: .3
//				//robot.drive.followTrajectory(moveToSpotThree);
//				//if parking 3 color
//				//drive to the left one tile
//				//drive backward to low terminal
//				//turn x degrees toward low terminal
//				//move lift to low terminal
//				//swing arm to above the low terminal
//				//release claw
//				//end program
//			}
//			else {
//				telemetry.addLine("Loser lol" );}
			}
		}
	}
}



