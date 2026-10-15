package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.DashboardBoolean
import frc.robot.utils.DashboardMeasure
import frc.robot.utils.angle
import frc.robot.utils.asMeters
import frc.robot.utils.clearUnwrappedDevice
import frc.robot.utils.convert
import frc.robot.utils.feetPerSecond
import frc.robot.utils.inches
import frc.robot.utils.radiansPerSecond
import frc.robot.utils.unwrappedDeviceAlerts
import java.io.File
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.encoders.SwerveAbsoluteEncoder
import swervelib.motors.SwerveMotor
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity

object Drivetrain : SubsystemBase() {
    object Constants {
        /** Tunable in NetworkTables (`Drivetrain/MAX_SPEED`, meters per second). */
        var MAX_SPEED by DashboardMeasure(Units.MetersPerSecond, 15.1.feetPerSecond, "Drivetrain")
        /** Tunable in NetworkTables (`Drivetrain/MAX_ANGULAR_SPEED`, radians per second). */
        var MAX_ANGULAR_SPEED by
            DashboardMeasure(Units.RadiansPerSecond, PI.radiansPerSecond, "Drivetrain")
        // Chassis configuration (left to right dist of center of the wheels)
        private val TRACK_WIDTH = 11.5.inches.asMeters

        // Distance between centers of right and left wheels on robot (front to back dist)
        private val WHEEL_BASE = 11.5.inches.asMeters

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DRIVE_KINEMATICS =
            arrayOf(
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")

        val ROBOT_WIDTH = 29.inches
        // val BUMPER_WIDTH = 35.inches
    }

    private val swerveDrive: SwerveDrive

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

    init {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects
        // being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH

        swerveDrive =
            SwerveParser(Constants.DRIVE_CONFIG)
                .createSwerveDrive(Constants.MAX_SPEED.convert(Units.MetersPerSecond))

        // Set YAGSL preferences
        swerveDrive.setHeadingCorrection(false)
        // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(true)
        // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations
        // since it causes discrepancies not seen in real life.
        swerveDrive.setMotorIdleMode(false)

        swerveDrive.setGyroOffset(Rotation3d(0.0, 0.0, 0.0))

        // Vision odometry is pushed by the Vision subsystem itself via
        // [addVisionMeasurement] (gated by Vision.enableYagslVision).
        setVisionMeasurementStdDevs(3.0, 4.0, 5.0)
        // setupPathPlanner()

        //        targetPoseProvider.initialize()

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
        for ((index, module) in swerveDrive.modules.withIndex()) {
            monitorMotor("Drivetrain/module$index/drive", module.driveMotor)
            monitorMotor("Drivetrain/module$index/angle", module.angleMotor)
            monitorEncoder("Drivetrain/module$index/encoder", module.absoluteEncoder)
        }
        Alerts.register(
            "Drivetrain/IMU",
            { runCatching { swerveDrive.gyro.clearStickyFaults() } },
        ) {
            imuAlerts()
        }
    }

    /** Registers a YAGSL drive/angle motor, dispatching on the wrapped vendor device. */
    private fun monitorMotor(label: String, motor: SwerveMotor) {
        Alerts.register(label, { clearUnwrappedDevice(runCatching { motor.motor }.getOrNull()) }) {
            val raw = runCatching { motor.motor }.getOrNull()
            val alerts = unwrappedDeviceAlerts(label, raw).toMutableList()
            if (alerts.isEmpty()) {
                // Unknown device type: fall back to a live-read probe.
                if (
                    runCatching {
                        motor.position
                        motor.velocity
                    }
                        .isFailure
                )
                    alerts += Alerts.Alert(label, "unreadable", Alerts.Level.ERROR)
            }
            alerts
        }
    }

    /** Registers a YAGSL absolute encoder, combining the read-error flag with vendor faults. */
    private fun monitorEncoder(label: String, encoder: SwerveAbsoluteEncoder) {
        Alerts.register(label, { runCatching { encoder.clearStickyFaults() } }) {
            val alerts = mutableListOf<Alerts.Alert>()
            if (runCatching { encoder.readingError }.getOrDefault(false))
                alerts += Alerts.Alert(label, "absolute encoder read issue", Alerts.Level.ERROR)
            val raw = runCatching { encoder.absoluteEncoder }.getOrNull()
            if (raw != null && runCatching { encoder.absolutePosition }.isFailure)
                alerts += Alerts.Alert(label, "unreadable", Alerts.Level.ERROR)
            alerts += unwrappedDeviceAlerts(label, raw)
            alerts
        }
    }

    /** Live IMU health: gyro probe plus wrapped-vendor faults. Empty when healthy. */
    private fun imuAlerts(): List<Alerts.Alert> {
        val alerts = mutableListOf<Alerts.Alert>()
        if (runCatching { swerveDrive.gyro.rotation3d }.isFailure)
            alerts += Alerts.Alert("Drivetrain/IMU", "unreadable", Alerts.Level.ERROR)
        alerts +=
            unwrappedDeviceAlerts(
                "Drivetrain/IMU",
                runCatching { swerveDrive.gyro.imu }.getOrNull(),
            )
        return alerts
    }

    override fun periodic() {
        imuOk = imuAlerts().none { it.level == Alerts.Level.ERROR }
        posePublisher.set(pose)
        swerveStatePublisher.set(swerveDrive.states)
        if (!enabled) {
            swerveDrive.drive(ChassisSpeeds())
            return
        }
        Vision.setAllCameraReferences(Pose3d(pose))
        SmartDashboard.putNumber("Odometry/X", pose.x)
        SmartDashboard.putNumber("Odometry/Y", pose.y)
        SmartDashboard.putNumber("Odometry/HEADING", pose.rotation.radians)
    }

    fun getAlliance(): DriverStation.Alliance =
        DriverStation.getAlliance().getOrNull() ?: DriverStation.Alliance.Blue

    /**
     * Drives field-oriented speeds. Field-oriented is always field-oriented: this never silently
     * degrades to robot-oriented. Callers that need a trustworthy heading must check
     * [fieldOrientedAvailable] first.
     */
    fun driveFieldOriented(speeds: ChassisSpeeds) {
        if (!enabled) {
            swerveDrive.drive(ChassisSpeeds())
            return
        }
        swerveDrive.driveFieldOriented(speeds)
    }

    fun driveRobotOriented(speeds: ChassisSpeeds) {
        if (!enabled) {
            swerveDrive.drive(ChassisSpeeds())
            return
        }
        swerveDrive.drive(speeds)
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
    fun driveLive(speeds: () -> ChassisSpeeds): Command =
        Commands.run({ driveFieldOriented(speeds()) }, this).until { !fieldOrientedAvailable }

    /**
     * Live command that drives robot-oriented [ChassisSpeeds] from a supplier. Requires the
     * drivetrain. Intended for full-axis auto-mode aligners.
     *
     * @param speeds supplier of the desired robot-oriented speeds, polled every cycle.
     */
    fun driveLiveRobotOriented(speeds: () -> ChassisSpeeds): Command =
        Commands.run({ driveRobotOriented(speeds()) }, this)

    /**
     * Return SysID command for drive motors from YAGSL
     *
     * @param spin whether to spin in place instead of driving straight (defaults to true,
     *   preserving the previous behavior).
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotors(spin: Boolean = true): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(),
                this,
                swerveDrive,
                12.0,
                spin,
            ),
            3.0,
            5.0,
            3.0,
        )
    }

    /**
     * Spin-in-place variant of [sysIdDriveMotors]: modules align tangentially and the robot rotates
     * about its center, so characterization needs no long linear area.
     *
     * @return A command that SysIDs the drive motors while spinning.
     */
    fun sysIdDriveMotorsSpin(): Command? = sysIdDriveMotors(spin = true)

    /**
     * Return SysID command for angle motors from YAGSL
     *
     * @return A command that SysIDs the angle motors.
     */
    fun sysIdAngleMotors(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(SysIdRoutine.Config(), this, swerveDrive),
            3.0,
            5.0,
            3.0,
        )
    }

    /**
     * Advanced drive method that translates and rotates the robot, with a custom center of
     * rotation.
     *
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field-oriented or robot-oriented.
     * @param centerOfRotation The center of rotation of the robot.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double = 0.0,
        fieldOriented: Boolean = false,
        centerOfRotation: Translation2d = Translation2d(),
    ) {
        if (!enabled) {
            swerveDrive.drive(ChassisSpeeds())
            return
        }
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot. Robot-oriented is always
     * robot-oriented.
     *
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds, fieldOriented: Boolean = false) {
        if (!enabled) {
            swerveDrive.drive(ChassisSpeeds())
            return
        }
        if (fieldOriented) swerveDrive.driveFieldOriented(velocity) else swerveDrive.drive(velocity)
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

    /** Returns the current pose of the robot. */
    val pose
        get() = Pose2d(swerveDrive.pose.x, swerveDrive.pose.y, swerveDrive.pose.rotation)

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

    /** Method to get the current heading (yaw) of the robot. */
    private val heading: Rotation2d
        get() = swerveDrive.yaw

    /**
     * Raw gyro yaw, or `null` when the subsystem is switched off or the IMU is unreadable.
     * Consumers must fall back (robot-oriented drive, pose-only heading) instead of assuming zero.
     */
    val rawYaw: Angle?
        get() =
            if (!fieldOrientedAvailable) null
            else runCatching { swerveDrive.gyro.rotation3d.toRotation2d().angle }.getOrNull()

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    @Suppress("unused")
    fun getTargetSpeeds(vForward: Double, vSide: Double, angle: Rotation2d): ChassisSpeeds =
        swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            angle.radians,
            heading.radians,
            Constants.MAX_SPEED.convert(Units.MetersPerSecond),
        )

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and angle X and Y components.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param headingX The desired X component of the angle.
     * @param headingY The desired Y component of the angle.
     * @return The generated ChassisSpeeds object.
     */
    @Suppress("unused")
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double,
    ): ChassisSpeeds =
        swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            headingX,
            headingY,
            heading.radians,
            Constants.MAX_SPEED.convert(Units.MetersPerSecond),
        )

    /** Returns the current field oriented velocity of the robot. */
    val fieldVelocity: ChassisSpeeds
        get() = swerveDrive.fieldVelocity

    /** Returns the current robot oriented velocity of the robot. */
    val robotVelocity: ChassisSpeeds
        get() = swerveDrive.robotVelocity

    /** Method to toggle the lock position of the swerve drive to prevent motion. */
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
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(stdDevX, stdDevY, stdDevTheta)
        )
    }
}
