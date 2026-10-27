package frc.robot.subsystems

import com.studica.frc.AHRS
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.Drivetrain.driveSysIdRoutine
import frc.robot.utils.DashboardBoolean
import frc.robot.utils.asMeters
import frc.robot.utils.clearUnwrappedDevice
import frc.robot.utils.degrees
import frc.robot.utils.degreesPerSecond
import frc.robot.utils.feetPerSecond
import frc.robot.utils.inches
import frc.robot.utils.radiansPerSecond
import frc.robot.utils.unwrappedDeviceAlerts
import java.io.File
import kotlin.math.PI
import swervelib.parser.SwerveParser
import yams.mechanisms.config.SwerveDriveConfig
import yams.mechanisms.swerve.SwerveDrive
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity
import yams.telemetry.SwerveDriveTelemetryConfig

object Drivetrain : SubsystemBase() {
    object Constants {
        val MAX_SPEED = 15.1.feetPerSecond
        val MAX_ANGULAR_SPEED = PI.radiansPerSecond
        // Chassis configuration (left to right dist of center of the wheels)
        private val TRACK_WIDTH = 11.5.inches.asMeters

        // Distance between centers of right and left wheels on robot (front to back dist)
        private val WHEEL_BASE = 11.5.inches.asMeters

        // Swerve module positions for [../Autos.kt]
        val DRIVE_KINEMATICS =
            arrayOf(
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")
    }

    /**
     * Master off-switch (NetworkTables `Drivetrain/enabled`). Set to `false` to run as a defense
     * bot: drive requests are ignored (outputs zeroed) and vision corrections are rejected, while
     * odometry telemetry keeps publishing.
     */
    var enabled: Boolean by DashboardBoolean(true, "Drivetrain")

    /** Whether the gyro is currently trustworthy; refreshed every cycle in [periodic]. */
    private var imuOk = true

    /** Whether field-oriented driving is safe (subsystem on and gyro healthy). */
    val fieldOrientedAvailable: Boolean
        get() = enabled && imuOk

    /** SwerveModuleStates publisher for swerve display */
    private val swerveStatePublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct)
            .publish()
    private val posePublisher: StructPublisher<Pose2d> =
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

    /**
     * NavX IMU. YAGSL 2026.9.17 dropped SPI NavX support from `swervedrive.json` (only `navx3_can`
     * is parsed), so it is supplied to the [SwerveDriveConfig] directly via
     * [SwerveDriveConfig.withGyro].
     */
    val navX = AHRS(AHRS.NavXComType.kMXP_SPI)

    private val config: SwerveDriveConfig =
        SwerveDriveConfig()
            .withSubsystem(this)
            // Negated: RIO is upside-down
            .withGyro { -navX.angle.degrees }
            .withGyroVelocity { -navX.rate.degreesPerSecond }
            .withGyroInverted(false)
            // .withGyroOffset(...), .withGyroVelocity(...) are also available
            // these controllers are used in [SwerveDrive.driveToPose]:
            .withTranslationController(PIDController(4.0, 0.0, 0.0))
            .withRotationController(PIDController(1.0, 0.0, 0.0))
            .withMaximumChassisSpeed(Constants.MAX_SPEED, Constants.MAX_ANGULAR_SPEED)
            .withTelemetry("swerve", SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH))

    private val swerveDevices: SwerveParser.SwerveDriveDevices
    private val swerveDrive: SwerveDrive

    init {
        // why does SwerveParser use global state?
        // the world may never know
        SwerveParser.parse(Constants.DRIVE_CONFIG)
        swerveDevices = SwerveParser.createSwerveDriveDevices(config)
        swerveDrive = swerveDevices.swerveDrive()

        // Vision odometry is pushed by the Vision subsystem itself via
        // [addVisionMeasurement] (gated by Vision.enableYagslVision).
        setVisionMeasurementStdDevs(3.0, 4.0, 5.0)

        registerHealth()
    }

    /**
     * Registers every swerve device (drive + angle motors, absolute encoders, IMU) with [Alerts],
     * plus a subsystem-level source for the off-switch.
     */
    private fun registerHealth() {
        Alerts.register("Drivetrain", {}) {
            if (enabled) emptyList()
            else listOf(Alerts.Alert("Drivetrain", "disabled by off-switch", Alerts.Level.WARNING))
        }
        // Module devices are in the same order as the JSON names in `swervedrive.json`.
        val moduleNames = SwerveParser.swerveDriveJson.modules
        for ((index, module) in swerveDevices.modules().withIndex()) {
            val name = moduleNames.getOrNull(index)?.substringBeforeLast('.') ?: "module$index"
            registerRawDevice("Drivetrain/$name/drive", module.drive())
            registerRawDevice("Drivetrain/$name/angle", module.azimuth())
            registerRawDevice("Drivetrain/$name/encoder", module.absoluteEncoder())
        }
        Alerts.register("Drivetrain/IMU", {}) { imuAlerts() }
    }

    /** Registers a raw vendor device (motor controller / encoder) with [Alerts]. */
    private fun registerRawDevice(label: String, raw: Any?) {
        Alerts.register(label, { clearUnwrappedDevice(raw) }) { unwrappedDeviceAlerts(label, raw) }
    }

    /** Live IMU health: gyro probe plus wrapped-vendor faults. Empty when healthy. */
    private fun imuAlerts(): List<Alerts.Alert> {
        val alerts = mutableListOf<Alerts.Alert>()
        if (runCatching { swerveDrive.gyroAngle }.isFailure)
            alerts += Alerts.Alert("Drivetrain/IMU", "unreadable", Alerts.Level.ERROR)
        alerts += unwrappedDeviceAlerts("Drivetrain/IMU", navX)
        return alerts
    }

    override fun periodic() {
        swerveDrive.updateTelemetry()
        imuOk = imuAlerts().none { it.level == Alerts.Level.ERROR }
        posePublisher.set(pose)
        swerveStatePublisher.set(swerveDrive.moduleStates)
        if (!enabled) {
            swerveDrive.setRobotRelativeChassisSpeeds(ChassisSpeeds())
            return
        }
        Vision.setAllCameraReferences(Pose3d(pose))
        SmartDashboard.putNumber("Odometry/X", pose.x)
        SmartDashboard.putNumber("Odometry/Y", pose.y)
        SmartDashboard.putNumber("Odometry/HEADING", pose.rotation.radians)
    }

    /**
     * Drives field-oriented speeds. Field-oriented is always field-oriented: this never silently
     * degrades to robot-oriented. Callers that need a trustworthy heading must check
     * [fieldOrientedAvailable] first.
     */
    fun driveFieldOriented(speeds: ChassisSpeeds) {
        if (!enabled) {
            swerveDrive.setRobotRelativeChassisSpeeds(ChassisSpeeds())
            return
        }
        swerveDrive.setFieldRelativeChassisSpeeds(speeds)
    }

    fun driveRobotOriented(speeds: ChassisSpeeds) {
        if (!enabled) {
            swerveDrive.setRobotRelativeChassisSpeeds(ChassisSpeeds())
            return
        }
        swerveDrive.setRobotRelativeChassisSpeeds(speeds)
    }

    fun stop() {
        driveRobotOriented(ChassisSpeeds())
    }

    /**
     * Live command that drives field-oriented [ChassisSpeeds] from a supplier. Requires the
     * drivetrain. Intended for full-axis auto-mode aligners.
     *
     * Ends immediately when field-oriented driving is unavailable (gyro untrustworthy): driving
     * field-frame on a bad heading is worse than not driving.
     *
     * @param speeds supplier of the desired field-oriented speeds, polled every cycle.
     */
    fun driveLive(speeds: () -> ChassisSpeeds): Command = run {
        driveFieldOriented(speeds())
    }.until { !fieldOrientedAvailable }

    /**
     * Live command that drives robot-oriented [ChassisSpeeds] from a supplier. Requires the
     * drivetrain. Intended for full-axis auto-mode aligners.
     *
     * @param speeds supplier of the desired robot-oriented speeds, polled every cycle.
     */
    fun driveLiveRobotOriented(speeds: () -> ChassisSpeeds): Command = run {
        driveRobotOriented(speeds())
    }

    /**
     * Generic WPILib SysId routine for the drive motors.
     *
     * Follows the WPILib "Creating an Identification Routine" pattern used by CTRE's swerve example
     * and described across ChiefDelphi: the drive callback sends the routine voltage to every drive
     * motor, and the log callback records per-module voltage, angular position, and angular
     * velocity (same signals YAMS logged in PR #73). Wheels are pre-aligned straight ahead so the
     * robot drives in a straight line, matching a translation characterization. Needs ~10-20 ft of
     * space per WPILib docs.
     */
    private val driveSysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage ->
                    swerveDrive.modules.forEach { it.driveMotorController.voltage = voltage }
                },
                { log ->
                    swerveDrive.modules.forEach { mod ->
                        log.motor(mod.driveMotorController.name)
                            .voltage(mod.driveMotorController.voltage)
                            .angularPosition(mod.driveMotorController.mechanismPosition)
                            .angularVelocity(mod.driveMotorController.mechanismVelocity)
                    }
                },
                this,
            ),
        )

    /**
     * Generic WPILib SysId routine for the azimuth (angle) motors.
     *
     * Same pattern as [driveSysIdRoutine] but driving every azimuth motor and logging its signals.
     */
    private val angleSysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage ->
                    swerveDrive.modules.forEach { it.azimuthMotorController.voltage = voltage }
                },
                { log ->
                    swerveDrive.modules.forEach { mod ->
                        log.motor(mod.azimuthMotorController.name)
                            .voltage(mod.azimuthMotorController.voltage)
                            .angularPosition(mod.azimuthMotorController.mechanismPosition)
                            .angularVelocity(mod.azimuthMotorController.mechanismVelocity)
                    }
                },
                this,
            ),
        )

    /** Align all azimuths straight ahead (0 rotations) for a drive-straight SysId run. */
    private fun alignModulesStraight() {
        swerveDrive.modules.forEach {
            it.azimuthMotorController.setPosition(Units.Rotations.of(0.0))
        }
    }

    /**
     * Align all azimuths tangentially (per-module angle for pure rotation) for a spin-in-place
     * SysId run. Same idea as the old `SwerveDriveTest` `testWithSpinning` option and YAMS PR #73
     * `DriveSysIdTestType.SPIN`: the robot rotates about its center instead of driving away, so no
     * long linear area is needed.
     */
    private fun alignModulesTangential() {
        val rotaryStates = swerveDrive.kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 1.0))
        swerveDrive.modules.forEachIndexed { i, mod ->
            mod.azimuthMotorController.setPosition(Units.Radians.of(rotaryStates[i].angle.radians))
        }
    }

    /**
     * Combined SysId command for drive motors (replaces removed `SwerveDriveTest`).
     *
     * Runs quasistatic forward/reverse then dynamic forward/reverse with settle delays, mirroring
     * the old `SwerveDriveTest.generateSysIdCommand(routine, 3.0, 5.0, 3.0)` timings. Stops the
     * drive closed-loop controllers for the run (required by YAMS before out-of-band `setVoltage`)
     * and restarts them afterward.
     *
     * @param spin if true, aligns modules tangentially so the robot spins in place (old
     *   `testWithSpinning` behavior) instead of driving in a straight line.
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotors(spin: Boolean = false): Command {
        val label = if (spin) "spin " else ""
        return Commands.print("Starting drive ${label}SysId!")
            .andThen(
                Commands.runOnce(
                    { if (spin) alignModulesTangential() else alignModulesStraight() },
                    this,
                )
            )
            .andThen(
                Commands.print("Waiting for wheels to align").andThen(Commands.waitSeconds(1.5))
            )
            .andThen(
                Commands.runOnce(
                    {
                        swerveDrive.modules.forEach {
                            it.driveMotorController.stopClosedLoopController()
                        }
                    },
                    this,
                )
            )
            .andThen(
                driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0)
            )
            .andThen(Commands.waitSeconds(3.0))
            .andThen(
                driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0)
            )
            .andThen(Commands.waitSeconds(3.0))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(3.0))
            .andThen(Commands.waitSeconds(3.0))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(3.0))
            .finallyDo(
                Runnable {
                    swerveDrive.modules.forEach {
                        it.driveMotorController.startClosedLoopController()
                    }
                }
            )
            .andThen(Commands.print("Done with drive ${label}SysId!"))
    }

    /**
     * Spin-in-place variant of [sysIdDriveMotors]: modules align tangentially and the robot rotates
     * about its center, so characterization needs no long linear area.
     *
     * @return A command that SysIDs the drive motors while spinning.
     */
    fun sysIdDriveMotorsSpin(): Command = sysIdDriveMotors(spin = true)

    /**
     * Combined SysId command for angle motors (replaces removed `SwerveDriveTest`).
     *
     * Same structure as [sysIdDriveMotors] but for the azimuth motors.
     *
     * @return A command that SysIDs the angle motors.
     */
    fun sysIdAngleMotors(): Command {
        return Commands.print("Starting azimuth SysId!")
            .andThen(
                Commands.runOnce(
                    {
                        swerveDrive.modules.forEach {
                            it.azimuthMotorController.stopClosedLoopController()
                        }
                    },
                    this,
                )
            )
            .andThen(
                angleSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0)
            )
            .andThen(Commands.waitSeconds(3.0))
            .andThen(
                angleSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0)
            )
            .andThen(Commands.waitSeconds(3.0))
            .andThen(angleSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(3.0))
            .andThen(Commands.waitSeconds(3.0))
            .andThen(angleSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(3.0))
            .finallyDo(
                Runnable {
                    swerveDrive.modules.forEach {
                        it.azimuthMotorController.startClosedLoopController()
                    }
                }
            )
            .andThen(Commands.print("Done with azimuth SysId!"))
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot. Robot-oriented is always
     * robot-oriented.
     *
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds, fieldOriented: Boolean = false) {
        if (!enabled) {
            swerveDrive.setRobotRelativeChassisSpeeds(ChassisSpeeds())
            return
        }
        if (fieldOriented) swerveDrive.setFieldRelativeChassisSpeeds(velocity)
        else swerveDrive.setRobotRelativeChassisSpeeds(velocity)
    }

    /** A ChassisSpeeds consumer used to drive the robot (Mainly for the purposes of PathPlanner) */
    val driveConsumer: (ChassisSpeeds) -> Unit = { fieldSpeeds: ChassisSpeeds ->
        drive(fieldSpeeds)
    }

    private var hasFieldPose = false

    /**
     * Method to reset the odometry of the robot to a desired pose.
     *
     * @param initialHolonomicPose The desired pose to reset the odometry to.
     */
    fun resetOdometry(initialHolonomicPose: Pose2d) {
        runCatching { swerveDrive.resetOdometry(initialHolonomicPose) }
    }

    /**
     * Returns the current pose of the robot, relative to the initial (init/reset) position. Always
     * available.
     */
    val pose: Pose2d
        get() = swerveDrive.pose

    /**
     * Returns the current pose of the robot relative to the field, or null until a vision
     * measurement tells us where on the field we are.
     */
    val fieldPose: Pose2d?
        get() = if (hasFieldPose) pose else null

    /** Method to zero the gyro. */
    fun zeroGyro() {
        runCatching { swerveDrive.zeroGyro() }
        hasFieldPose = false
    }

    /**
     * Raw gyro yaw, or `null` when the subsystem is switched off or the IMU is unreadable.
     * Consumers must fall back (robot-oriented drive, pose-only heading) instead of assuming zero.
     */
    val rawYaw: Angle?
        get() =
            if (!fieldOrientedAvailable) null else runCatching { swerveDrive.gyroAngle }.getOrNull()

    /** Returns the current field oriented velocity of the robot. */
    val fieldVelocity: ChassisSpeeds
        get() = swerveDrive.fieldRelativeSpeed

    /** Returns the current robot oriented velocity of the robot. */
    val robotVelocity: ChassisSpeeds
        get() = swerveDrive.robotRelativeSpeed

    /** Method to enable the lock position of the swerve drive to prevent motion. */
    fun lock() {
        if (!enabled) return
        swerveDrive.lockPose()
    }

    /**
     * Add a vision measurement to the swerve drive's pose estimator. Called by the Vision subsystem
     * (gated by Vision.enableYagslVision).
     *
     * @param measurement The pose measurement to add.
     * @param timestamp The timestamp of the pose measurement.
     */
    fun addVisionMeasurement(
        measurement: Pose2d,
        timestamp: Double,
        updateRotation: Boolean = true,
    ) {
        if (!enabled) return
        hasFieldPose = true
        if (updateRotation) swerveDrive.addVisionMeasurement(measurement, timestamp)
        else
            swerveDrive.addVisionMeasurement(
                Pose2d(measurement.x, measurement.y, pose.rotation),
                timestamp,
            )
    }

    /**
     * Set the standard deviations of the vision measurements.
     *
     * @param stdDevX The standard deviation of the X component of the vision measurements.
     * @param stdDevY The standard deviation of the Y component of the vision measurements.
     * @param stdDevTheta The standard deviation of the rotational component of the vision
     *   measurements.
     */
    @Suppress("SameParameterValue")
    private fun setVisionMeasurementStdDevs(stdDevX: Double, stdDevY: Double, stdDevTheta: Double) {
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevX, stdDevY, stdDevTheta))
    }
}
