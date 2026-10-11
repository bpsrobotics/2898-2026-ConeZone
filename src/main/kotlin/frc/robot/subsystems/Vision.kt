package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.DashboardBoolean
import frc.robot.utils.degrees
import frc.robot.utils.inches
import frc.robot.utils.registerConnectedSensor
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult

object Vision : SubsystemBase() {
    /** A single PhotonVision camera and its pose estimator. */
    class Camera
    internal constructor(
        val name: String,
        val robotToCamera: Transform3d,
        private val cam: PhotonCamera,
        private val poseEstimator: PhotonPoseEstimator,
    ) {
        var referencePose: Pose3d = Pose3d()

        val isConnected: Boolean
            get() = cam.isConnected

        val results: List<PhotonPipelineResult>
            get() = cam.allUnreadResults

        /** Estimates the robot pose from a result, falling back to the reference pose. */
        fun estimatePose(result: PhotonPipelineResult): Pose3d? {
            // A disconnected camera's numbers are unreliable: report null so consumers fall back
            // (ignore the measurement) instead of trusting stale data.
            if (!isConnected) return null
            if (result.targets.isEmpty()) return null
            val estimation =
                if (result.multiTagResult.isPresent)
                    poseEstimator.estimateCoprocMultiTagPose(result)
                else poseEstimator.estimateClosestToReferencePose(result, referencePose)
            if (estimation.isEmpty) return null
            return estimation.get().estimatedPose
        }
    }

    /** Disposable handle for a result subscription; call [close] to unsubscribe. */
    class Subscription internal constructor(private val onClose: () -> Unit) : AutoCloseable {
        override fun close() = onClose()
    }

    private val statePublisher: StringPublisher =
        NetworkTableInstance.getDefault().getStringTopic("Vision/State").publish()

    /** Cameras that failed to initialize, each reported once. */
    private val failedCameras = mutableSetOf<String>()

    /** Cameras already reported as missing, so we only complain once per state change. */
    private val reportedMissing = mutableSetOf<String>()

    /**
     * Whether pose estimates should be pushed into YAGSL's pose estimator. The driver's gyro-reset
     * hold turns this off mid-match when vision goes bad, leaving odometry relative to the zeroed
     * heading; reset at disabled init.
     */
    var enablePoseEstimation: Boolean by DashboardBoolean(true, "Vision")

    /**
     * Master off-switch (NetworkTables `Vision/enabled`). Set to `false` to run as a defense bot:
     * camera results are ignored entirely (estimates read as `null`), while connection state keeps
     * publishing.
     */
    var enabled: Boolean by DashboardBoolean(true, "Vision")

    private val resultSubscribers = mutableListOf<(PhotonPipelineResult, Camera) -> Unit>()

    private val fieldLayout: AprilTagFieldLayout? =
        try {
            AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("conezone.json"))
        } catch (e: Exception) {
            reportProblem("Failed to load the Cone Zone AprilTag field layout", e)
            null
        }

    /** The configured cameras; may be empty if the field layout or a camera failed to load. */
    val cameras: List<Camera> =
        fieldLayout?.let { layout ->
            listOfNotNull(
                createCamera(
                    "Iris_Arducam",
                    Transform3d(
                        (-1.021).inches, // -13
                        5.276.inches,
                        19.010.inches,
                        Rotation3d(0.0.degrees, (-30.0).degrees, 30.degrees),
                    ),
                    layout,
                ),
                createCamera(
                    "Retina_Arducam",
                    Transform3d(
                        (-1.021).inches, // -13
                        (-5.276).inches,
                        19.010.inches,
                        Rotation3d(0.0.degrees, (-30.0).degrees, (-30).degrees),
                    ),
                    layout,
                ),
            )
        } ?: emptyList()

    init {
        updateState()
        registerHealth()
    }

    override fun periodic() {
        updateState()
        if (!enabled) return
        for (camera in cameras) {
            for (result in camera.results) {
                if (enablePoseEstimation) addVisionMeasurement(camera, result)
                for (subscriber in resultSubscribers) subscriber(result, camera)
            }
        }
    }

    /** Registers a handler to run for every camera result. */
    fun onResult(handler: (PhotonPipelineResult, Camera) -> Unit): Subscription {
        resultSubscribers += handler
        return Subscription { resultSubscribers -= handler }
    }

    /** Updates the reference pose used by each camera's fallback pose strategy. */
    fun setAllCameraReferences(pose: Pose3d) {
        for (camera in cameras) camera.referencePose = pose
    }

    private fun createCamera(
        name: String,
        robotToCamera: Transform3d,
        layout: AprilTagFieldLayout,
    ): Camera? =
        try {
            Camera(
                name,
                robotToCamera,
                PhotonCamera(name),
                PhotonPoseEstimator(layout, robotToCamera),
            )
        } catch (e: Exception) {
            failedCameras += name
            reportProblem("Failed to initialize camera '$name'", e)
            null
        }

    private fun addVisionMeasurement(camera: Camera, result: PhotonPipelineResult) {
        if (!enabled) return
        if (result.targets.isEmpty()) return
        if (!result.multiTagResult.isPresent && result.targets.first().poseAmbiguity > 0.3) return
        val pose = camera.estimatePose(result)?.toPose2d() ?: return
        Drivetrain.addVisionMeasurement(pose, result.timestampSeconds)
    }

    /** Registers the subsystem and every camera with [Alerts]. Cameras can't clear faults. */
    private fun registerHealth() {
        Alerts.register("Vision", {}) {
            when {
                !enabled ->
                    listOf(Alerts.Alert("Vision", "disabled by off-switch", Alerts.Level.WARNING))
                !enablePoseEstimation ->
                    listOf(
                        Alerts.Alert(
                            "Vision",
                            "odometry feed disabled, pose is gyro-relative",
                            Alerts.Level.WARNING,
                        )
                    )
                failedCameras.isNotEmpty() ->
                    failedCameras.sorted().map {
                        Alerts.Alert(
                            "Vision",
                            "camera '$it' failed to initialize",
                            Alerts.Level.ERROR,
                        )
                    }
                cameras.isEmpty() ->
                    listOf(Alerts.Alert("Vision", "no cameras configured", Alerts.Level.ERROR))
                else -> emptyList()
            }
        }
        for (camera in cameras) {
            Alerts.registerConnectedSensor(
                "Vision/${camera.name}",
                isConnected = { camera.isConnected },
            )
        }
    }

    private fun updateState() {
        val disconnected = cameras.filterNot { it.isConnected }.map { it.name }
        statePublisher.set(
            when {
                failedCameras.isNotEmpty() -> "ERROR: ${failedCameras.sorted().joinToString()}"
                cameras.isEmpty() -> "NO CAMERAS"
                disconnected.isEmpty() -> "OK"
                else -> "MISSING: ${disconnected.sorted().joinToString()}"
            }
        )
        for (name in disconnected) {
            if (reportedMissing.add(name)) {
                DriverStation.reportWarning("Vision: camera '$name' is not connected", false)
            }
        }
        reportedMissing.retainAll(disconnected.toSet())
    }

    private fun reportProblem(message: String, error: Exception) {
        DriverStation.reportError("Vision: $message: ${error.message}", false)
        statePublisher.set("ERROR: $message")
    }
}
