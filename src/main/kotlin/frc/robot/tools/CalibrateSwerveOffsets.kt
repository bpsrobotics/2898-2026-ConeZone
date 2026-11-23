package frc.robot.tools

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEvent
import java.nio.file.Files
import java.nio.file.Path
import java.util.EnumSet
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

private val modules = listOf("frontleft", "frontright", "backleft", "backright")
private val offsetPattern = Regex("(\"absoluteEncoderOffset\"\\s*:\\s*)(-?\\d+(?:\\.\\d+)?)")

fun main(args: Array<String>) {
    require(args.size == 3) { "Expected robot host, sample seconds, and module directory" }
    val host = args[0]
    val seconds = args[1].toDouble()
    require(seconds > 0 && seconds <= 60) { "sampleSeconds must be in (0, 60]" }
    val directory = Path.of(args[2])
    val nt = NetworkTableInstance.create()
    nt.startClient4("swerve-offset-calibration")
    nt.setServer(host)
    val subscribers = modules.associateWith { name ->
        nt.getDoubleTopic("/Mechanisms/swerve/modules/$name/encoder").subscribe(Double.NaN)
    }
    val latest = modules.associateWith { AtomicReference(Double.NaN) }
    val listeners = subscribers.map { (name, subscriber) ->
        nt.addListener(subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll)) {
            latest.getValue(name).set(it.valueData.value.double)
        }
    }
    try {
        val waitUntil = System.nanoTime() + 10_000_000_000L
        while (System.nanoTime() < waitUntil && latest.values.any { !it.get().isFinite() }) Thread.sleep(50)
        check(latest.values.all { it.get().isFinite() }) { "Timed out waiting for all four encoder topics" }

        val sums = modules.associateWith { DoubleArray(3) }
        val end = System.nanoTime() + (seconds * 1_000_000_000L).toLong()
        var previous = System.nanoTime()
        while (System.nanoTime() < end) {
            Thread.sleep(20)
            check(nt.isConnected) { "NetworkTables connection lost during sampling" }
            val now = minOf(System.nanoTime(), end)
            val duration = (now - previous) / 1_000_000_000.0
            for ((name, value) in latest) {
                val angle = value.get()
                check(angle.isFinite()) { "Invalid encoder angle for $name" }
                val radians = Math.toRadians(angle)
                sums.getValue(name).apply {
                    this[0] += sin(radians) * duration
                    this[1] += cos(radians) * duration
                    this[2] += duration
                }
            }
            previous = now
        }

        val updates = mutableMapOf<Path, String>()
        for ((name, sum) in sums) {
            check(sum[2] >= seconds * 0.95 && hypot(sum[0], sum[1]) / sum[2] >= 0.95) {
                "Insufficient or unstable readings for $name"
            }
            val angle = (Math.toDegrees(atan2(sum[0], sum[1])) + 360) % 360
            val file = directory.resolve("$name.json")
            val content = Files.readString(file)
            val match = offsetPattern.find(content)
                ?: error("Missing absoluteEncoderOffset in $file")
            updates[file] = content.replaceRange(match.range, "${match.groupValues[1]}$angle")
            println("$name: $angle degrees (${String.format("%.2f", sum[2])} seconds)")
        }
        for ((file, content) in updates) Files.writeString(file, content)
    } finally {
        listeners.forEach { nt.removeListener(it) }
        subscribers.values.forEach { it.close() }
        nt.close()
    }
}
