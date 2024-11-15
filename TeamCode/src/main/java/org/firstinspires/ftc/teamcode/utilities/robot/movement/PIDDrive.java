package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class PIDDrive {
    //  GeneralPIDController xController = new GeneralPIDController(0.3, 0, 0, 0);
    // GeneralPIDController yController = new GeneralPIDController(0.3, 0, 0, 0);

    GeneralPIDController xController = new GeneralPIDController(0.2, 0, 1, 0);
    GeneralPIDController yController = new GeneralPIDController(0.2, 0, 1, 0);
    GeneralPIDController headingController = new GeneralPIDController(1.5, 0, 0, 0);

    public static double vMax = 50;
    public static double aMax = 40;

    public static double kA = 0.0035;
    public static double kV = 0.013;

    public static Pose threshold = new Pose(new Coordinate(0.25, 0.25), Math.toRadians(2));
    public static double thresholdTime = 1;

    RobotEx theRobot;

    Telemetry theTelemetry;
    LinearOpMode theOpMode;

    public PIDDrive(RobotEx aRobot, LinearOpMode anOpMode, Telemetry aTelemetry) {
        theRobot = aRobot;
        theOpMode = anOpMode;
        theTelemetry = aTelemetry;
    }


    public void gotoPoint(Pose point) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, DriveConstants.MAX_CORRECTION_TIME)
        );
    }

    public void gotoPoint(Pose point, double correctionTime) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, correctionTime)
        );
    }


    //TODO. This is a WIP. Lots to redo here I think.
    //DOES NOT WORK AS IS WITH ALL THE CHANGES MADE. Will fix

    public void gotoPoint(Pose aFinalLocation, MovementConstants constants) {

        Pose currentPose = theRobot.theOpticalOdometry.getPose();

        Coordinate myCoordinateDifference = new Coordinate(aFinalLocation.getCoordinate());
        myCoordinateDifference.substract(currentPose.getCoordinate());

        Pose error = new Pose(
                myCoordinateDifference,
                aFinalLocation.getTheFinalDirection() - currentPose.getTheFinalDirection()
        );

        double displacement = myCoordinateDifference.magnitude();

        MotionProfile motion = new MotionProfile(
                0, displacement, constants.velocityMax, constants.accelerationMax
        );

        double angle = Math.atan2(myCoordinateDifference.getY(), myCoordinateDifference.getX());
        double sineTerm = Math.sin(angle);
        double cosineTerm = Math.cos(angle);

        double duration = motion.getDuration();
        double currentProfileTime = 0;

        boolean inPosition = false;

        ElapsedTime inPositionTime = new ElapsedTime();
        ElapsedTime profileTime = new ElapsedTime();

        while (!theRobot.theStopRequested) {

            currentPose = theRobot.theOpticalOdometry.getPose();
            currentProfileTime = profileTime.seconds();

            double targetDisplacement = motion.getPositionFromTime(currentProfileTime);
            double xTarget = cosineTerm * targetDisplacement + startPosition.getX();
            double yTarget = sineTerm * targetDisplacement + startPosition.getY();


            double headingTarget = MathHelper.lerp(startPosition.getTheFinalDirection(), aFinalLocation.getTheFinalDirection(), Math.min(currentProfileTime + 0.05, duration) / duration);

            error = new Pose(
                    xTarget - currentPose.getX(),
                    yTarget - currentPose.getY(),
                    headingTarget - currentPose.getTheFinalDirection()
            );


            if (Math.abs(error.getTheFinalDirection()) > Math.PI) {
                error.setTheFinalDirection(
                        AngleHelper.norm(headingTarget) - AngleHelper.norm(currentPose.getTheFinalDirection())
                );
            }

            double currentAcceleration = motion.getAccelerationFromTime(currentProfileTime);
            double currentVelocityAmount = motion.getVelocityFromTime(currentProfileTime);
            double feedforward = motion.getAccelerationFromTime(currentProfileTime) * kA + motion.getVelocityFromTime(currentProfileTime) * kV;

            double feedforwardX = feedforward * sineTerm;
            double feedforwardY = feedforward * cosineTerm;

            double feedbackX = xController.getOutputFromError(error.getX());
            double feedbackY = yController.getOutputFromError(error.getY());

            theRobot.theDrivetrain.fieldCentricDriveFromGamepad(
                    feedforwardX + feedbackY,
                    feedforwardY + feedbackX,
                    -MathUtils.clamp(headingController.getOutputFromError(
                            error.getTheFinalDirection()
                    ), -0.75, 0.75)
            );

            // telemetry.addData("Profile time: ", currentProfileTime);
            // telemetry.addData("Motion time: ", duration);
            // telemetry.addData("Error h: ", error.getHeading());

            theTelemetry.addData("Target Velocity X: ", currentVelocityAmount * sineTerm);
            theTelemetry.addData("Target Velocity Y: ", currentVelocityAmount * cosineTerm);

            /*
            telemetry.addData("Error y: ", error.getY());
            telemetry.addData("feedback x: ", feedbackX);
            telemetry.addData("feedback y: ", feedbackY);

             */

            theRobot.update();
            error.map(Math::abs);
            currentVelocity = new Pose(currentVelocity).map(Math::abs);

            if (error.lessThan(threshold) && currentProfileTime > duration) {
                if (inPosition) {
                    if (inPositionTime.seconds() >= thresholdTime || currentVelocity.lessThan(threshold)) {
                        break;
                    }
                } else {
                    inPosition = true;
                    inPositionTime.reset();
                }
            } else if (currentProfileTime > duration + constants.maxCorrectionTime) {
                break;
            } else {
                inPosition = false;
            }

        }

        theRobot.theDrivetrain.fieldCentricDriveFromGamepad(
                0,
                0,
                0
        );

        theRobot.update();
    }


    public void turnToAngle(double angle, MovementConstants constants) {
        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = theRobot.theOpticalOdometry.getPose().getTheFinalDirection();
        double turnError;

        MotionProfile turnProfile = new MotionProfile(currentIMUPosition, angle, DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && elapsedTurnTime.seconds() < turnProfile.getDuration() + constants.maxCorrectionTime) {

            double currentTargetAngle = turnProfile.getPositionFromTime(elapsedTurnTime.seconds());
            turnError = currentTargetAngle - currentIMUPosition;

            if (Math.abs(turnError) > Math.PI) {
                if (angle < 0) {
                    angle = AngleHelper.norm(angle);
                    turnError = angle - currentIMUPosition;
                } else if (angle > 0) {
                    currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                    turnError = angle - currentIMUPosition;
                }
            }

            double output = theRobot.theDrivetrain.profiledTurningPID.getOutputFromError(
                    turnError
            );


            theRobot.theDrivetrain.robotCentricDriveFromGamepad(
                    0,
                    0,
                    -Math.min(Math.max(output, -1), 1) + Math.signum(output)
            );

            currentIMUPosition = theRobot.theOpticalOdometry.getPose().getTheFinalDirection();

            if (theTelemetry != null) {
                theTelemetry.addData("Target Angle: ", currentTargetAngle);
                theTelemetry.addData("Turn Angle: ", turnError);
                theTelemetry.addData("current angle: ", currentIMUPosition);
                theTelemetry.update();
            }

            this.theRobot.update();
        }

        MotorGroup<DcMotorEx> robotDrivetrain = theRobot.theDrivetrain.getTheDriveTrainMotorGroup();

        DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = theRobot.theDrivetrain.getZeroPowerBehavior();

        if (currentZeroPowerBehavior != DcMotor.ZeroPowerBehavior.BRAKE) {
            robotDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotDrivetrain.setPower(0);
            theRobot.pause(0.1);
            robotDrivetrain.setZeroPowerBehavior(currentZeroPowerBehavior);
        }


    }

    public void turnToAngle(double angle) {
        turnToAngle(angle, new MovementConstants(0, 0, DriveConstants.MAX_CORRECTION_TIME));
    }
}
