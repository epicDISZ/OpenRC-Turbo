package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
DONE: Fix servo delayed start
DONE: Fix straight drive for all modes
DONE: Lift Height For lvl 2
DONE: Automatic servo for Micro
DONE: Automatic shooting for Macro
TODO: Possible small changes to syntax for Macro
DONE: Start lift at height of 33
TODO-DONE: Adjust servo open time in micro pol system
TODO: Fix buttons for gamepad2
TODO: Long press (~300ms) for lift start height
 */

@TeleOp(name="Drive", group="Drive")
public class Drive extends OpMode {
    //Constants
    private final int ticksPerMicroRev = 778;
    private final int[] rgbaUpper = new int[] {2100, 650, 480, 3500};
    private final int[] rgbaLower = new int[] {2700, 1000, 590, 3900};
    private final double microMaxDistance = 5.8;
    private final double normalSpeed = 0.6;
    private final double slowSpeed = 0.25;
    private final double fastSpeed = 0.9;
    private final double lowerLiftBound = 9;
    private final double upperLiftBound = 36.3;
    private final double liftLockHeight = 20.9;
    private final double startLiftHeight = 32;
    private final int ticksForLoad = 1120;

    //Variables
    private double driveSpeed;
    private double liftHeight;
    private boolean constIntake;
    private boolean straightDrive;
    private boolean liftAtStart;
    private boolean manualShoot;

    //Robot States
    private DriveState driveState;
    private MicroState microState;
    private MacroState macroState;

    //Robot Hardware
    private DcMotorEx rightMotor;
    private DcMotorEx forRight;
    private DcMotorEx leftMotor;
    private DcMotorEx forLeft;
    private DcMotor intakeMotor;
    private DcMotorEx microPolMotor;
    private DcMotorEx macroPolMotor;
    private DcMotorEx liftMotor;
    private Servo liftLock;
    private Servo macroTrigger;
    private Servo microGate;

    private ElapsedTime microRuntime = new ElapsedTime();
    private ElapsedTime macroRuntime = new ElapsedTime();

    private Rev2mDistanceSensor liftDistanceSensor;
    private ColorSensor microColorSensor;
    private DistanceSensor microDistanceSensor;
    private TouchSensor macroMagLimit;

    //Telemetry
    Telemetry.Item teleSpeed;
    Telemetry.Item teleMicroState;
    Telemetry.Item teleMacroState;
    Telemetry.Item teleLiftHeight;

    @Override
    public void init() {
        driveState = DriveState.Normal;
        driveSpeed = normalSpeed;
        constIntake = false;
        straightDrive = false;
        liftAtStart = false;
        manualShoot = false;
        microState = MicroState.Idle;
        macroState = MacroState.Idle;

        //Initialize all motors and Servos
        rightMotor = hardwareMap.get(DcMotorEx.class, "RightMotor");
        forRight = hardwareMap.get(DcMotorEx.class, "ForRight");
        leftMotor = hardwareMap.get(DcMotorEx.class, "LeftMotor");
        forLeft = hardwareMap.get(DcMotorEx.class, "ForLeft");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        microPolMotor = hardwareMap.get(DcMotorEx.class, "MicroPolMotor");
        macroPolMotor = hardwareMap.get(DcMotorEx.class, "MacroPolMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        liftLock = hardwareMap.get(Servo.class, "LiftLock");
        macroTrigger = hardwareMap.get(Servo.class, "MacroTrigger");
        microGate = hardwareMap.get(Servo.class, "MicroGate");
        //Initialize all sensors
        liftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "LiftDistance");
        microColorSensor = hardwareMap.get(ColorSensor.class, "MicroColorSensor");
        microDistanceSensor = hardwareMap.get(DistanceSensor.class, "MicroColorSensor");
        macroMagLimit = hardwareMap.get(TouchSensor.class, "MacroMagLimit");
        //TODO: Add reverses
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        forRight.setDirection(DcMotor.Direction.REVERSE);
        microPolMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //TODO: Add brake button
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        microPolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        macroPolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        microPolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        macroPolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        microPolMotor.setTargetPosition(0);
        macroPolMotor.setTargetPosition(0);
        microPolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        macroPolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        microPolMotor.setPower(1);
        macroPolMotor.setPower(1);
        //TODO: Test default targetPositionTolerance
        microPolMotor.setTargetPositionTolerance(4);
        macroPolMotor.setTargetPositionTolerance(4);

        //Initialize Servos
        liftLock.setPosition(0);
        macroTrigger.setPosition(0);
        microGate.setPosition(0.1);
        //Initialize Telemetry
        teleSpeed = telemetry.addData("Drive Speed", driveSpeed);
        teleMicroState = telemetry.addData("Micro Pol State", microState);
        teleMacroState = telemetry.addData("Macro Pol State", macroState);
        teleLiftHeight = telemetry.addData("Lift Height", liftHeight);
    }

    @Override
    public void start() {
        super.start();
        constIntake = true;
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger)
            driveState = DriveState.Slow;
        else if (gamepad1.left_trigger)
            driveState = DriveState.Fast;
        else driveState = DriveState.Normal;

        if (gamepad1.left_bumper)
            straightDrive = true;
        else straightDrive = false;

        //Sets Drive Speed Based on driveState
        driveSpeed = (driveState == DriveState.Slow) ? slowSpeed :
                        (driveState == DriveState.Fast) ? fastSpeed : normalSpeed;
        if (straightDrive)
            setMotor(-gamepad1.left_stick_y, -gamepad1.left_stick_y, driveSpeed);  //Temp code
        else
            setMotor(-gamepad1.left_stick_y, -gamepad1.right_stick_y, driveSpeed); //Temp code

        //Activate constant intake if right_stick_y is bigger than 0.8
        constIntake = (0.9 < gamepad2.right_stick_y && gamepad2.right_stick_button) || constIntake;
        //Disable constant intake if right_stick_y is under -0.5
        constIntake = (!(gamepad2.right_stick_y < -0.5)) && constIntake;
        //Sets motor based on constant intake or right_stick_y
        intakeMotor.setPower((constIntake) ? 1 : gamepad2.right_stick_y);

        liftHeight = liftDistanceSensor.getDistance(DistanceUnit.CM); //Get distance from liftSensor

        //Get lift to start position before giving up control
        if (!liftAtStart) {
            if (liftHeight <= startLiftHeight - 0.2)
                liftMotor.setPower(-1);
            else if (startLiftHeight + 0.2 <= liftHeight)
                liftMotor.setPower(1);
            else {
                liftMotor.setPower(0);
                liftAtStart = true;
            }
        }
        else {
            //Limit lift to the lift's bounds
            if ((lowerLiftBound < liftHeight && gamepad2.left_stick_y > 0) ||
                    (liftHeight < upperLiftBound && gamepad2.left_stick_y < 0)) //TODO: Correct sticks and optimize.
                liftMotor.setPower(gamepad2.left_stick_y); //TODO: invert gamepad stick, correct motor direction.
            else liftMotor.setPower(0);
            //Let the lift locking servo extend when the lift is at the proper height
            if (liftHeight < liftLockHeight && gamepad2.left_trigger)
                liftLock.setPosition(1);
            if (gamepad2.x)
                liftLock.setPosition(0);
        }
        if (gamepad2.left_stick_button)
            liftAtStart = false;

        switch (microState) {
            case Idle:
                if (gamepad2.a)
                    microState = MicroState.StartFeed;
                break;
            case StartFeed:
                microGate.setPosition(0.26);
                microState = MicroState.Feeding;
                microRuntime.reset();
                break;
            case Feeding:
                if (305 < microRuntime.milliseconds()) {
                    microGate.setPosition(0.1);
                    microState = MicroState.StartShoot;
                    microRuntime.reset();
                }
                break;
            case StartShoot:
                if (checkColor(microColorSensor, rgbaUpper, rgbaLower) ||
                        microDistanceSensor.getDistance(DistanceUnit.CM) <= microMaxDistance) {
                    microPolMotor.setTargetPosition(microPolMotor.getTargetPosition() + ticksPerMicroRev); //Change to target position to allow feeding before shooting has finished
                    microState = MicroState.Shooting;
                }
                else if (3000 < microRuntime.milliseconds())
                    microState = microState.Idle;
                break;
            case Shooting:
                if (microPolMotor.getTargetPosition() - ((ticksPerMicroRev / 4) * 3) < microPolMotor.getCurrentPosition()) {
                    microPolMotor.setTargetPosition(microPolMotor.getTargetPosition());
                    microState = MicroState.Idle;
                }
                break;
            default: microState = MicroState.Idle; break;
        }

        //TODO: Check position of macroMotor when shooting.

        switch (macroState) {
            case Idle:
//                if (gamepad2.a)
//                    macroState = MacroState.StartLoading;
                macroState = (gamepad2.y) ? MacroState.StartLoading : macroState;
                break;
            case StartLoading:
                macroPolMotor.setTargetPosition(ticksForLoad + 250);
                macroState = MacroState.Loading;
                break;
            case Loading:
                if (ticksForLoad < macroPolMotor.getCurrentPosition())
                    macroState = MacroState.StartLock;
                break;
            case StartLock:
                macroTrigger.setPosition(0.75);
                macroState = MacroState.Locked;
                macroRuntime.reset();
                break;
            case Locked:
                if (700 < macroRuntime.milliseconds()) {
                    macroPolMotor.setTargetPosition(0);
                    macroState = MacroState.LockedAndLoaded;
                }
                break;
            case LockedAndLoaded:
                if ((gamepad2.b) && macroPolMotor.getCurrentPosition() < 15)
                    macroState = MacroState.Shoot;
                break;
            case Shoot:
                macroTrigger.setPosition(0);
                macroState = MacroState.Idle;
                break;
            default: macroState = MacroState.Idle; break;
        }

        /*if (macroMagLimit.isPressed())
            macroTrigger.setPosition(0.7);
        if (gamepad2.left_stick_button && gamepad2.right_stick_button)
            macroTrigger.setPosition(0);*/

        /*if (gamepad2.right_bumper)
            macroPolMotor.setPower(1);
        else if (gamepad2.left_bumper)
            macroPolMotor.setPower(-1);
        else macroPolMotor.setPower(0);*/

        if (gamepad2.dpad_down)
            microGate.setPosition(0.1);
        else if (gamepad2.dpad_up)
            microGate.setPosition(0.26);

        if (gamepad2.right_trigger && !manualShoot) {
            microPolMotor.setTargetPosition(microPolMotor.getTargetPosition() + ticksPerMicroRev);
        }
        else if (manualShoot){
            if (microPolMotor.getTargetPosition() - 10 < microPolMotor.getCurrentPosition()) {
                manualShoot = false;
            }
        }

        teleSpeed.setValue(driveSpeed);
        teleMicroState.setValue(microState);
        teleMacroState.setValue(macroState);
        teleLiftHeight.setValue(liftHeight);
        telemetry.update();
    }

    public static boolean checkColor(ColorSensor colorSensor, int[] rgbaUpper, int[] rgbaLower) {
        int[] rgba = new int[] {colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()};
        boolean compareResults = true;
        for (int i = 0; i <= 3; i++) {
            compareResults = (rgbaLower[i] <= rgba[i] && rgba[i] <= rgbaUpper[i]) && compareResults;
        }
        return compareResults;
    }

    //TODO: Make possibly make static, make the function take in a array of motors.
    //More: Passing through objects in java passes pointer to memory, as long as we don't = a object it stays the reference.
    //https://stackoverflow.com/a/40523/6122159
    public void setMotor(double leftStick, double rightStick, double multiplier) { //Temporary function!
        rightMotor.setPower(rightStick * multiplier);
        forRight.setPower(rightStick * multiplier);
        leftMotor.setPower(leftStick * multiplier);
        forLeft.setPower(leftStick * multiplier);
    }

    //Enums / States

    public enum DriveState {
        Normal,
        Fast,
        Slow
    }

    public enum MicroState {
        Idle,
        StartFeed,
        Feeding,
        StartShoot,
        Shooting
    }

    public enum MacroState {
        Idle,
        StartLoading,
        Loading,
        StartLock,
        Locked,
        LockedAndLoaded,
        Shoot
    }
}

