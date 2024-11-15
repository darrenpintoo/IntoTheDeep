package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

/**
 * Robot Drivetrain
 */
// LF - 0
// RF - 2
// RB - 1
// LB - 3
@Config
public class Drivetrain implements Subsystem {

    public enum TurnDirection {
        LEFT, RIGHT
    }

    private RobotEx theRobotInstance;

    MotorGroup<DcMotorEx> theDriveTrainMotorGroup;
    private DcMotorEx[] theDrivetrainMotors;

    public DcMotorEx theRightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;


    private boolean enableAntiTip = false;
    private boolean enableHeadingRetention = false;

    public GeneralPIDController headingPID = new GeneralPIDController(1.5, 0, 20, 0);
    public GeneralPIDController profiledTurningPID = new GeneralPIDController(1.5, 0, 40, 0);

    public GeneralPIDController translationalPID = new GeneralPIDController(1, 0, 0, 0);

    public GeneralPIDController tiltPID = new GeneralPIDController(0.3, 0, 0, 0);
    public GeneralPIDController yawPID = new GeneralPIDController(0.3, 0, 0, 0);

    private Telemetry telemetry;

    private double rightFrontPower = 0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private double lastRightFrontPower = 0;
    private double lastLeftFrontPower = 0;
    private double lastLeftBackPower = 0;
    private double lastRightBackPower = 0;


    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;

    private double weight = 1;

    public static double kP = 2;
    public static double kI = 0;
    public static double kD = 40;

    private double trackWidth = 12;
    private double wheelBase = 6.5;
    private double lateralMultiplier = -1.2;

    public static double LATERAL_MULTIPLIER = 1.33;

    @Override
    public void onInit(HardwareMap aHardwareMap, Telemetry aTelemetry) {

        this.theRobotInstance = RobotEx.getInstance();

        theRightFrontMotor = new CachingDcMotorEX((DcMotorEx) aHardwareMap.get(DcMotor.class, "rightFrontMotor"), 1e-5);
        leftFrontMotor = new CachingDcMotorEX((DcMotorEx) aHardwareMap.get(DcMotor.class, "leftFrontMotor"), 1e-5);
        leftBackMotor = new CachingDcMotorEX((DcMotorEx) aHardwareMap.get(DcMotor.class, "leftBackMotor"), 1e-5);
        rightBackMotor = new CachingDcMotorEX((DcMotorEx) aHardwareMap.get(DcMotor.class, "rightBackMotor"), 1e-5);

        // todo: figure out the directions

        theDrivetrainMotors = new DcMotorEx[] {
                this.theRightFrontMotor,
                this.leftFrontMotor,
                this.leftBackMotor,
                this.rightBackMotor
        };


        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        theRightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);


        theDriveTrainMotorGroup = new MotorGroup<>(
                this.theRightFrontMotor,
                this.leftFrontMotor,
                this.leftBackMotor,
                this.rightBackMotor
        );

        rightBackPower = 0;
        leftBackPower = 0;
        leftFrontPower = 0;
        rightFrontPower = 0;

        telemetry = aTelemetry;
    }

    public void enableAntiTip() {
        this.enableAntiTip = true;
    }

    public void disableAntiTip() {
        this.enableAntiTip = false;
    }

    public void enableHeadingRetention() {
        this.enableHeadingRetention = true;
    }

    public void disableHeadingRetention() {
        this.enableHeadingRetention = false;
    }

    @Override
    public void onOpmodeStarted() {
        this.theRobotInstance = RobotEx.getInstance();
    }

    @Override
    public void onCyclePassed() {

        this.headingPID.updateCoefficients(Drivetrain.kP, Drivetrain.kI, Drivetrain.kD, 0);

        this.rightBackMotor.setPower(rightBackPower);
        this.theRightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.leftFrontMotor.setPower(leftFrontPower);

        this.lastRightBackPower = this.rightBackPower;
        this.lastLeftBackPower= this.leftBackPower;
        this.lastLeftFrontPower = this.leftFrontPower;
        this.lastRightFrontPower = this.rightFrontPower;

        this.rightBackPower = 0;
        this.leftBackPower = 0;
        leftFrontPower = 0;
        rightFrontPower = 0;
    }

    public void robotCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {

        leftJoystickX *= LATERAL_MULTIPLIER;

        double multiple = RobotEx.getInstance().getPowerMultiple();

        leftJoystickY = MathHelper.clamp(leftJoystickY * multiple, -1, 1);
        leftJoystickX = MathHelper.clamp(leftJoystickX * multiple, -1, 1);
        rightJoystickX = MathHelper.clamp(rightJoystickX * multiple, -1, 1);

        double denominator = Math.max(Math.abs(leftJoystickY) + Math.abs(leftJoystickX) + Math.abs(rightJoystickX), 1);
        this.leftFrontPower += (leftJoystickY + leftJoystickX + rightJoystickX) / denominator;
        this.leftBackPower += (leftJoystickY - leftJoystickX + rightJoystickX) / denominator;
        this.rightFrontPower += (leftJoystickY - leftJoystickX - rightJoystickX) / denominator;
        this.rightBackPower += (leftJoystickY + leftJoystickX - rightJoystickX) / denominator;

        this.lastX = leftJoystickX;
        this.lastY = leftJoystickY;
        this.lastRot = rightJoystickX;
    }

    public void fieldCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        double currentRobotOrientation = theRobotInstance.theOpticalOdometry.currentPose.getTheFinalDirection() - Math.PI / 2;

        this.robotCentricDriveFromGamepad(
                Math.sin(-currentRobotOrientation) * leftJoystickX + Math.cos(-currentRobotOrientation) * leftJoystickY,
                (Math.cos(-currentRobotOrientation) * leftJoystickX - Math.sin(-currentRobotOrientation) * leftJoystickY),
                rightJoystickX
        );
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior newZeroPowerBehavior) {
        for (DcMotorEx drivetrainMotor : this.theDrivetrainMotors) {
            drivetrainMotor.setZeroPowerBehavior(newZeroPowerBehavior);
        }
    }

    public void setRunMode(DcMotor.RunMode newRunMode) {
        for (DcMotorEx drivetrainMotor : this.theDrivetrainMotors) {
            drivetrainMotor.setMode(newRunMode);
        }
    }

    public DcMotorEx[] getTheDrivetrainMotors() {
        return this.theDrivetrainMotors;
    }

    public MotorGroup<DcMotorEx> getTheDriveTrainMotorGroup() {
        return this.theDriveTrainMotorGroup;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.leftBackMotor.getZeroPowerBehavior();
    }


    public int[] getCWMotorTicks() {
        return new int[] {
                this.theRightFrontMotor.getCurrentPosition(),
                this.leftFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        };
    }

    public MecanumWheelState getMotorTicks() {
        return new MecanumWheelState(
                theRightFrontMotor.getCurrentPosition(),
                leftFrontMotor.getCurrentPosition(),
                leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()
        );
    }

    public MecanumWheelState getMotorVelocity() {
        return new MecanumWheelState(
                theRightFrontMotor.getVelocity(),
                leftFrontMotor.getVelocity(),
                leftBackMotor.getVelocity(),
                rightBackMotor.getVelocity()
        );
    }

    public void setWeightedDrivePower(double weight) {
        this.weight = weight;
    }


    public double getTrackWidth() {
        return trackWidth;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public double getLateralMultiplier() {
        return lateralMultiplier;
    }
}

