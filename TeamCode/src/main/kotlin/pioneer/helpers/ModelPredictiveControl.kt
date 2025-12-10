package pioneer.helpers

import java.util.concurrent.Callable
import java.util.concurrent.Executors
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class ModelPredictiveControl(
    private val horizon: Int = 10,
    private val maxAccel: Double = 50.0,
    private val posWeight: Double = 1.0,
    private val controlWeight: Double = 0.01,
    private val iterations: Int = 3,
) {
    private val threadPool = Executors.newFixedThreadPool(6)

    fun update(
        current: Pose,
        target: Pose,
        dt: Double,
    ): Pose {
        val initialControl = computeInitialControl(current, target, dt)
        return optimize(current, target, initialControl, dt)
    }

    fun shutdown() = threadPool.shutdown()

    private fun optimize(
        current: Pose,
        target: Pose,
        initControl: Pose,
        dt: Double,
    ): Pose {
        var best = initControl
        var bestCost = evaluateCost(current, target, best, dt)

        repeat(iterations) { iter ->
            val delta = maxAccel * 0.2 / (1 + iter * 0.5)

            val perturbations = listOf(
                Callable { Pair(best.copy(ax = best.ax + delta), 0) },
                Callable { Pair(best.copy(ax = best.ax - delta), 1) },
                Callable { Pair(best.copy(ay = best.ay + delta), 2) },
                Callable { Pair(best.copy(ay = best.ay - delta), 3) },
                Callable { Pair(best.copy(alpha = best.alpha + delta), 4) },
                Callable { Pair(best.copy(alpha = best.alpha - delta), 5) },
            )

            val futures = threadPool.invokeAll(perturbations)
            
            futures.forEach { future ->
                val (control, _) = future.get()
                val clamped = clampControl(control)
                val c = evaluateCost(current, target, clamped, dt)
                if (c < bestCost) {
                    best = clamped
                    bestCost = c
                }
            }
        }

        return best
    }

    private fun computeInitialControl(current: Pose, target: Pose, dt: Double): Pose {
        val dt2 = dt * dt
        val dTheta = wrapAngle(target.theta - current.theta)
        val ax = ((target.x - current.x - current.vx * dt) / dt2).coerceIn(-maxAccel, maxAccel)
        val ay = ((target.y - current.y - current.vy * dt) / dt2).coerceIn(-maxAccel, maxAccel)
        val alpha = ((dTheta - current.omega * dt) / dt2).coerceIn(-maxAccel, maxAccel)
        return Pose(ax = ax, ay = ay, alpha = alpha)
    }

    private fun evaluateCost(current: Pose, target: Pose, control: Pose, dt: Double): Double {
        var pose = current
        var sum = 0.0

        repeat(horizon) {
            pose = pose.copy(
                vx = pose.vx + control.ax * dt,
                vy = pose.vy + control.ay * dt,
                omega = pose.omega + control.alpha * dt,
                ax = control.ax,
                ay = control.ay,
                alpha = control.alpha,
            ).integrate(dt)

            val posError = pose.distanceTo(target)
            val angError = abs(wrapAngle(pose.theta - target.theta))
            val controlEffort = control.ax * control.ax + control.ay * control.ay + control.alpha * control.alpha

            sum += posWeight * (posError + angError) + controlWeight * controlEffort
        }

        return sum
    }

    private fun clampControl(control: Pose): Pose =
        control.copy(
            ax = control.ax.coerceIn(-maxAccel, maxAccel),
            ay = control.ay.coerceIn(-maxAccel, maxAccel),
            alpha = control.alpha.coerceIn(-maxAccel, maxAccel),
        )

    private fun wrapAngle(angle: Double): Double = atan2(sin(angle), cos(angle))
}

