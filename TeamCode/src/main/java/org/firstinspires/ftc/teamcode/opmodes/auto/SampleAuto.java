package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "Sample Preload Auto")
public class SampleAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx theRobotInstance = RobotEx.getInstance();

    //USE BETTER NAMES FOR THESE LOCATIONS THAT REPRESENT WHAT THEY ARE
    //MOVE TO A NEW FILE IF USED IN MULTIPLE LOCATIONS

    public static final Coordinate START_LOCATION = new Coordinate(-37.6, -61.8);
    public static final Coordinate LOCATION_1= new Coordinate(-54, -55);
    public static final Coordinate LOCATION_2 = new Coordinate(-48.2, -52.2);
    public static final Coordinate LOCATION_3 = new Coordinate(-57.6, -47.8);
    public static final Coordinate LOCATION_4 = new Coordinate(-40, -10);
    public static final Coordinate LOCATION_5 = new Coordinate(-30, -10);
    public static final Coordinate LOCATION_6 = new Coordinate(-44, -27);


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        theRobotInstance.init(this, telemetry);


        waitForStart();

        theRobotInstance.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);

        theRobotInstance.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_DEFAULT);
        // Notify subsystems before loop
        theRobotInstance.postInit();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        // robot.drivetrain.enableAntiTip();

        theRobotInstance.update();

        ElapsedTime e = new ElapsedTime();

        // robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);

        ElapsedTime timer = new ElapsedTime();

        PIDDrive drive = new PIDDrive(theRobotInstance, this, telemetry);

        //Initial position. Placed manually.
        theRobotInstance.theOpticalOdometry.setPose(new Pose(START_LOCATION, Math.PI / 2));
        theRobotInstance.pause(0.05);

        drive.gotoPoint(new Pose(LOCATION_1, Math.PI / 4));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_2, Math.PI / 2));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_1, Math.PI / 4));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_3, Math.PI / 2));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_1, Math.PI / 4));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_6, Math.PI));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_1, Math.PI / 4));
        theRobotInstance.pause(2);
        drive.gotoPoint(new Pose(LOCATION_4, 0));
        drive.gotoPoint(new Pose(LOCATION_5, Math.PI / 4), new MovementConstants(10, 10, 0));






        while (!isStopRequested()) {
            theRobotInstance.update();
        }















        while (!isStopRequested()) {
            theRobotInstance.update();
        }





    }
}
