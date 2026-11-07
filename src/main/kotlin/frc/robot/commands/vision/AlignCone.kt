package frc.robot.commands.vision

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.MovingAverage
import frc.robot.utils.asRadians
import frc.robot.utils.controls.toPID
import frc.robot.utils.convert
import frc.robot.utils.degrees
import frc.robot.utils.getAlliance
import frc.robot.utils.radiansPerSecond
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sign

private val alignConeRotationPID = Drivetrain.Constants.ROTATION_PID.toPID()

/** How long the last seen cone keeps steering the robot after the pipeline stops reporting it. */
private const val ALIGN_CONE_TARGET_TIMEOUT_SECONDS = 0.15

/** Time for the camera to settle after switching pipelines before its targets are trusted. */
private const val ALIGN_CONE_PIPELINE_SETTLE_SECONDS = 0.15

/** Number of recent target-yaw samples to average when smoothing measurement jitter. */
private const val ALIGN_CONE_YAW_WINDOW = 3

private const val ALIGN_CONE_AREA_THRESHOLD = 12

/**
 * Adjusting the circularity threshold:
 * - A perfect circle yields 1.0.
 * - A perfect square yields 0.785 (pi / 4).
 * - An elongated tape strip or tube segment yields much lower values (e.g., 0.2 to 0.5).
 * - A threshold of 0.80 will filter out circular game pieces or camera lens flares, while keeping
 *   rectangular retroreflective targets or AprilTag profiles.
 */
private const val CONE_CIRCULARITY_THRESHOLD = 0.785

data class ChordResult(
    val maxDistance: Double,
    val point1: TargetCorner,
    val point2: TargetCorner,
)

/**
 * Finds the longest chord (diameter) of a convex polygon represented by PhotonVision TargetCorners.
 * Uses the Rotating Calipers algorithm to achieve O(n) time complexity.
 *
 * @param corners A list of target corners ordered circularly around the contour perimeter.
 * @return A ChordResult containing the maximum distance and the two corresponding corners.
 */
private fun findLongestChord(corners: List<TargetCorner>): ChordResult? {
    val n = corners.size
    if (n < 2) return null
    if (n == 2) {
        val dist = distance(corners[0], corners[1])
        return ChordResult(dist, corners[0], corners[1])
    }

    var maxDist = 0.0
    var bestP1 = corners[0]
    var bestP2 = corners[0]

    // Step 1: Find the initial antipodal pair.
    // Start with vertex 0 and find the vertex farthest away from the edge (0 -> 1)
    var k = 1
    while (
        triangleArea(corners[0], corners[1], corners[(k + 1) % n]) >
            triangleArea(corners[0], corners[1], corners[k])
    ) {
        k = (k + 1) % n
    }

    // Step 2: Rotate the calipers
    var i = 0
    var j = k
    while (i <= k && j < n) {
        // Check the current antipodal pair distance
        val dist = distance(corners[i], corners[j])
        if (dist > maxDist) {
            maxDist = dist
            bestP1 = corners[i]
            bestP2 = corners[j]
        }

        // Determine which caliper side to rotate by comparing triangle areas
        // formed with the next step vertices.
        val nextI = (i + 1) % n
        val nextJ = (j + 1) % n

        if (
            triangleArea(corners[i], corners[nextI], corners[nextJ]) >
                triangleArea(corners[i], corners[nextI], corners[j])
        ) {
            j = nextJ
        } else {
            i = nextI
        }
    }

    return ChordResult(maxDist, bestP1, bestP2)
}

/** Calculates the Euclidean distance between two TargetCorners. */
private fun distance(p1: TargetCorner, p2: TargetCorner): Double {
    return hypot(p1.x - p2.x, p1.y - p2.y)
}

/**
 * Computes twice the area of the triangle formed by three points. Equivalently, the cross product
 * magnitude showing relative distance from the baseline.
 */
private fun triangleArea(p1: TargetCorner, p2: TargetCorner, p3: TargetCorner): Double {
    return abs((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x))
}

/**
 * Finds whether a target cone is upright based on the edge-bounding polygon. First checks if the
 * shape is circular, if so, we're seeing the base or top, and it's tipped over. Then it checks the
 * angle of the longest chord of the polygon to see if that is upright.
 */
private fun targetIsUpright(target: PhotonTrackedTarget): Boolean {
    val corners = target.detectedCorners
    var totalArea = 0.0
    var totalPerimeter = 0.0
    val n = corners.size

    for (i in 0..<n) {
        val current = corners[i]
        val next = corners[(i + 1) % n]

        // Shoelace formula for Area
        totalArea += (current.x * next.y) - (next.x * current.y)

        // Summing sides for Perimeter
        totalPerimeter += distance(current, next)
    }
    val area = abs(totalArea) / 2.0
    val circularity =
        if (totalPerimeter > 0) (4.0 * PI * area) / (totalPerimeter * totalPerimeter) else 0.0
    if (circularity > CONE_CIRCULARITY_THRESHOLD) return false

    val chord = findLongestChord(corners) ?: return false
    val angle =
        Translation2d(chord.point2.x - chord.point1.x, chord.point2.y - chord.point1.y)
            .angle
            .measure
    return (45.degrees <= angle && angle <= 135.degrees) ||
        (225.degrees <= angle && angle <= 315.degrees)
}

/**
 * Turns the robot toward the largest cone of the current alliance color. The AR0234 pipeline is
 * switched to red or blue for the duration and restored to the driver feed afterward. Runs until
 * the cone is centered for a few frames, or until interrupted.
 */
fun alignCone(
    pipelineSupplier: () -> Vision.ConePipeline = {
        Vision.ConePipeline.forAlliance(getAlliance())
    },
    slowFactor: () -> Double = { 1.0 },
    maxRotSpeed: AngularVelocity = 1.0.radiansPerSecond,
): Command {
    val rotationPID = alignConeRotationPID
    rotationPID.enableContinuousInput(-PI, PI)
    val timer = Timer()
    val maxOmega = maxRotSpeed.convert(Units.RadiansPerSecond)

    var subscription: Vision.Subscription? = null
    var cone: PhotonTrackedTarget? = null
    var timeSinceConeSeen = 0.0
    var alignedFrames = 0
    // Smooths measurement jitter on the cone's yaw. Small window keeps feedback-loop lag low.
    val yawFilter = MovingAverage(ALIGN_CONE_YAW_WINDOW)
    val areaFilter = MovingAverage(ALIGN_CONE_YAW_WINDOW)

    fun computeSpeeds(): ChassisSpeeds {
        val target = cone
        if (target == null || timer.get() - timeSinceConeSeen > ALIGN_CONE_TARGET_TIMEOUT_SECONDS) {
            return ChassisSpeeds()
        }

        var rotationSpeed = rotationPID.calculate(yawFilter.average).coerceIn(-maxOmega, maxOmega)
        val deadzone = 0.2
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += deadzone * rotationSpeed.sign

        var forwardSpeed = if (areaFilter.average < ALIGN_CONE_AREA_THRESHOLD) 1.0 else 0.0

        rotationSpeed *= slowFactor()
        forwardSpeed *= slowFactor()

        return ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed)
    }

    return Drivetrain.driveLiveRobotOriented(::computeSpeeds)
        .beforeStarting(
            {
                timer.restart()
                Vision.pipeline = pipelineSupplier()
                subscription = Vision.onConeResult { result, _ ->
                    // The camera takes a few frames to adjust after a pipeline switch; ignore
                    // targets until it has settled.
                    if (timer.get() < ALIGN_CONE_PIPELINE_SETTLE_SECONDS) {
                        return@onConeResult
                    }
                    // The pipelines are largest-first, but pick explicitly so ordering changes are
                    // harmless.
                    val largest =
                        result.targets.filter { targetIsUpright(it) }.maxByOrNull { it.area }
                            ?: return@onConeResult
                    val now = timer.get()
                    val yaw = largest.yaw.degrees.asRadians
                    // Restart the filter when the cone is (re)acquired after a gap so stale samples
                    // don't pull the estimate toward a previously-seen position.
                    val reacquired =
                        cone == null || now - timeSinceConeSeen > ALIGN_CONE_TARGET_TIMEOUT_SECONDS
                    if (reacquired) {
                        yawFilter.clear(yaw)
                        areaFilter.clear(largest.area)
                    } else {
                        yawFilter.add(yaw)
                        areaFilter.add(largest.area)
                    }
                    cone = largest
                    timeSinceConeSeen = now
                }
                rotationPID.reset()
                cone = null
                alignedFrames = 0
            },
            Vision,
        )
        .until {
            val aligned =
                cone != null &&
                    yawFilter.average.absoluteValue < 2.degrees.asRadians &&
                    areaFilter.average > ALIGN_CONE_AREA_THRESHOLD
            if (aligned) alignedFrames += 1 else alignedFrames = 0
            alignedFrames > 3
        }
        .finallyDo(
            Runnable {
                subscription?.close()
                Vision.pipeline = Vision.ConePipeline.DRIVER
                Drivetrain.stop()
            }
        )
}
