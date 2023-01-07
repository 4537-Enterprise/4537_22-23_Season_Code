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
	String coneColor;
	Trajectory moveToCone;
	Trajectory ditchCone;
	Trajectory moveToSpotOne;
	Trajectory moveToSpotTwo;
	Trajectory backUp;
	Trajectory moveToSpotThree;
	Trajectory creep;

	enum TrajectoryState{
		MOVE_TO_CONE,
		READ_CONE,
		DITCH_CONE,
		MOVE_TO_SPOT_ONE,
		MOVE_TO_SPOT_TWO,
		BACK_UP,
		MOVE_TO_SPOT_THREE,
		IDLE
	}


	TrajectoryState trajectoryState = TrajectoryState.MOVE_TO_CONE;
	TrajectoryState desiredState;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	Pose2d currPose = new Pose2d(0, 0, Math.toRadians(0.0));

	@Override
	public void runOpMode() throws InterruptedException{
		robot = new CompRobot(hardwareMap, telemetry);

		// Define our start pose
		robot.drive.setPoseEstimate(currPose);

		// Initial trajectory
		moveToCone = robot.drive.trajectoryBuilder(currPose)
				.back(14)
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

				case READ_CONE:
					coneColor = robot.colorSensor.getConeColor();
					if (coneColor == "Fuchsia"){
						desiredState = TrajectoryState.MOVE_TO_SPOT_ONE;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					if (coneColor == "Cyan"){
						desiredState = TrajectoryState.MOVE_TO_SPOT_TWO;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					if (coneColor == "Yellow"){
						desiredState = TrajectoryState.MOVE_TO_SPOT_THREE;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					if (coneColor == "Unknown"){
						creep = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(1).build();
						robot.drive.followTrajectory(creep);
					}
					break;

				case DITCH_CONE:
					ditchCone = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(11).build();
					robot.drive.followTrajectory(ditchCone);
					this.trajectoryState = desiredState;
					break;

				case MOVE_TO_SPOT_ONE:
					moveToSpotOne = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeRight(22).build();
					robot.drive.followTrajectory(moveToSpotOne);
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case MOVE_TO_SPOT_TWO:
					moveToSpotTwo = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(28).build();
					robot.drive.followTrajectory(moveToSpotTwo);
					backUp = robot.drive.trajectoryBuilder(moveToSpotTwo.end()).forward(4).build();
					robot.drive.followTrajectory(backUp);
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case MOVE_TO_SPOT_THREE:
					moveToSpotThree = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(22).build();
					robot.drive.followTrajectory(moveToSpotThree);
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case IDLE:
					break;
			}

//				//then move backward x amount to target high terminal
//				//turn x amount of degrees to face the high terminal
//				// move lift up to high terminal
//				//swing arm above terminal
//				//release claw
//				//end program

//				//then drive backward to two-three inches past the high terminal
//				//drive back to high terminal
//				//turn x degrees to high terminal
//				//move lift to high terminal
//				//swing arm to above terminal
//				//release claw
//				//end program

//				//if parking 3 color
//				//drive to the left one tile
//				//drive backward to low terminal
//				//turn x degrees toward low terminal
//				//move lift to low terminal
//				//swing arm to above the low terminal
//				//release claw
//				//end program
//			}

		}
	}
}




