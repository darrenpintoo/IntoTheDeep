package org.firstinspires.ftc.teamcode.utilities.robot;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.OpticalOdometry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;


import java.util.List;

/*
Robot Configuration
Control Hub
    Motors:
        0 - rightBackMotor
        1 - rightFrontMotor
        2 - leftFrontMotor
        3 - leftBackMotor
    Servos:
        0 - intakeClaw
        1 - boxOpen
        2 - rightBox
        3 - leftBox
        4 - leftIntakeServo
        5 - rightIntakeServo
    DPIO
        0/1 - breakBeams
        2/3 - leftProximity
        4/5 - centerProximity
        6/7 - rightProximity
Expansion Hub
    Motors:
    Servos:

 */
public class RobotEx {
    private static RobotEx theRobot = null;

    List<LynxModule> theHubList;

    public Drivetrain theDrivetrain = new Drivetrain();
    public OpticalOdometry theOpticalOdometry = new OpticalOdometry();
    public Outtake theOuttake = new Outtake();
    public Intake theIntake = new Intake();
    public Limelight theLimelight = new Limelight();

    public VoltageSensor theVoltageSensor;

    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] theRobotSubsystems = new Subsystem[]{
            theDrivetrain,
            theOpticalOdometry,
            theOuttake,
            theIntake,
            theLimelight
    };

    Telemetry theTelemetry;

    public HardwareMap theHardwareMap;

    public LinearOpMode theOpMode;

    public boolean theStopRequested = false;
    public double theRunTime = 0;

    private double theVoltageCompensator = 12;
    private double theFrames = 0;
    private double theCurrentFrames = 0;
    private double theLastTime = 0;


    //Code to ensure only a single instance of RobotEx created.
    private RobotEx() {
        if (RobotEx.theRobot != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static RobotEx getInstance() {
        if (RobotEx.theRobot == null) {
            RobotEx.theRobot = new RobotEx();
        }

        return RobotEx.theRobot;
    }



    public void init(LinearOpMode anOpMode, Telemetry aTelemetry) {
        this.theOpMode = anOpMode;
        this.theHardwareMap = anOpMode.hardwareMap;
        this.theTelemetry = aTelemetry;

        this.theVoltageSensor = theHardwareMap.voltageSensor.iterator().next();
        this.theVoltageCompensator = this.theVoltageSensor.getVoltage();

        for (Subsystem subsystem : this.theRobotSubsystems) {
            subsystem.onInit(theHardwareMap, aTelemetry);
        }

        aTelemetry.update();
    }

    public void init(LinearOpMode anOpMode) {
        this.init(anOpMode, anOpMode.telemetry);
    }

    public void postInit() {

        theStopRequested = theOpMode.isStopRequested();

        if (theStopRequested) return;

        this.theHubList = theHardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : theHubList) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (Subsystem subsystem : this.theRobotSubsystems) {
            subsystem.onOpmodeStarted();
        }

    }

    @SuppressLint("")
    public double update() {

        theStopRequested = theOpMode.isStopRequested();

        if (theStopRequested) return 0;

        theRunTime = theOpMode.getRuntime();

        if (Math.floor(theRunTime) != theLastTime) {
            theFrames = theCurrentFrames;
            theCurrentFrames = 0;
            theLastTime = Math.floor(theRunTime);
        }

        theCurrentFrames += 1;

        ElapsedTime log = new ElapsedTime();

        log.reset();

        double startTime = frameTimer.milliseconds();

        for (LynxModule hub : theHubList) {
            hub.clearBulkCache();
        }

        theTelemetry.addData("End Cache Clear time: ", frameTimer.milliseconds() - startTime);


        for (Subsystem subsystem : theRobotSubsystems) {
            subsystem.onCyclePassed();
            theTelemetry.addData("Subsystem Update: ", frameTimer.milliseconds() - startTime);
        }

        theTelemetry.addLine("Refresh Rate: " + theFrames + " hz");
        theTelemetry.addData("Run time: ", theRunTime);

        theTelemetry.update();

        double frameTime = frameTimer.milliseconds();
        frameTimer.reset();

        return frameTime;
    }

    public void pause(double seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.seconds() < seconds && !theStopRequested) {
            update();
        }
    }

    public void persistData() {
        PersistentData.startPose = this.theOpticalOdometry.getPose();
    }

    public double getVoltage() {
        return this.theVoltageSensor.getVoltage();
    }

    public double getPowerMultiple() {
        return 12 / this.theVoltageCompensator;
    }

    public void destroy() {
        RobotEx.theRobot = null;
    }



}