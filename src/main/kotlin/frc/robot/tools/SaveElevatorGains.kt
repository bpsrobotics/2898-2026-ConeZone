package frc.robot.tools

import edu.wpi.first.networktables.NetworkTableInstance
import java.nio.file.Files
import java.nio.file.Path
import org.jetbrains.kotlin.cli.jvm.compiler.EnvironmentConfigFiles
import org.jetbrains.kotlin.cli.jvm.compiler.KotlinCoreEnvironment
import org.jetbrains.kotlin.com.intellij.openapi.util.Disposer
import org.jetbrains.kotlin.com.intellij.psi.util.PsiTreeUtil
import org.jetbrains.kotlin.config.CompilerConfiguration
import org.jetbrains.kotlin.psi.KtCallExpression
import org.jetbrains.kotlin.psi.KtProperty
import org.jetbrains.kotlin.psi.KtPsiFactory

private data class Gains(val topic: String, val properties: List<String>, val start: Int, val end: Int)

private fun markedGains(file: Path, factory: KtPsiFactory): List<Gains> {
    val source = factory.createFile(file.fileName.toString(), Files.readString(file))
    return PsiTreeUtil.findChildrenOfType(source, KtProperty::class.java).mapNotNull { property ->
        val annotation = property.annotationEntries.singleOrNull {
            it.shortName?.asString() == "LiveGains"
        } ?: return@mapNotNull null
        val arguments = annotation.valueArguments.map { it.getArgumentExpression()?.text.orEmpty() }
        check(arguments.size == 2 && arguments.all { it.startsWith('"') && it.endsWith('"') }) {
            "Invalid @LiveGains on ${property.name} in $file"
        }
        val topic = arguments[0].removeSurrounding("\"")
        val properties = arguments[1].removeSurrounding("\"").split(',').map(String::trim)
        val initializer = property.initializer as? KtCallExpression
            ?: error("@LiveGains requires a constructor call on ${property.name} in $file")
        val valueArguments = initializer.valueArgumentList
            ?: error("Missing gain constructor arguments on ${property.name} in $file")
        check(properties.size == valueArguments.arguments.size && properties.all(String::isNotBlank)) {
            "@LiveGains properties do not match constructor arguments on ${property.name} in $file"
        }
        Gains(topic, properties, valueArguments.textRange.startOffset, valueArguments.textRange.endOffset)
    }
}

fun main(args: Array<String>) {
    require(args.size == 2) { "Expected robot host and Kotlin source directory" }
    val root = Path.of(args[1])
    val disposable = Disposer.newDisposable()
    val sources = try {
        val environment = KotlinCoreEnvironment.createForProduction(
            disposable, CompilerConfiguration(), EnvironmentConfigFiles.JVM_CONFIG_FILES
        )
        val factory = KtPsiFactory(environment.project)
        Files.walk(root).use { paths ->
            paths.filter { Files.isRegularFile(it) && it.toString().endsWith(".kt") }
                .map { it to markedGains(it, factory) }
                .filter { it.second.isNotEmpty() }
                .toList()
        }
    } finally {
        Disposer.dispose(disposable)
    }
    check(sources.isNotEmpty()) { "No @LiveGains annotations found under $root" }
    val topics = sources.flatMap { (_, groups) ->
        groups.flatMap { group -> group.properties.map { "${group.topic}/$it" } }
    }
    check(topics.size == topics.toSet().size) { "Duplicate @LiveGains topic" }

    val nt = NetworkTableInstance.create()
    nt.startClient4("save-live-gains")
    nt.setServer(args[0])
    val subscribers = topics.associateWith { nt.getDoubleTopic(it).subscribe(Double.NaN) }
    try {
        val deadline = System.nanoTime() + 10_000_000_000L
        while (System.nanoTime() < deadline && subscribers.values.any { !it.get().isFinite() }) {
            Thread.sleep(50)
        }
        check(nt.isConnected) { "Not connected to robot at ${args[0]}" }
        val values = subscribers.mapValues { (topic, subscriber) ->
            subscriber.get().also { check(it.isFinite()) { "Missing live $topic" } }
        }
        val updates = sources.associate { (file, groups) ->
            val text = StringBuilder(Files.readString(file))
            for (group in groups.sortedByDescending(Gains::start)) {
                val numbers = group.properties.joinToString(", ") {
                    values.getValue("${group.topic}/$it").toString()
                }
                text.replace(group.start, group.end, "($numbers)")
            }
            file to text.toString()
        }
        updates.forEach { (file, content) -> Files.writeString(file, content) }
        topics.forEach { println("$it = ${values.getValue(it)}") }
    } finally {
        subscribers.values.forEach { it.close() }
        nt.close()
    }
}
