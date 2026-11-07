package com.arroute.nav.core

import kotlin.math.abs
import kotlin.math.hypot

class Route(val points: List<Vec2>) {
    init { require(points.size >= 2) { "A route needs at least two points." } }

    val length: Double = points.zipWithNext { a, b -> (b - a).length() }.sum()

    data class Projection(val distanceAlong: Double, val offset: Double)

    fun project(p: Vec2): Projection {
        var best = Projection(0.0, Double.MAX_VALUE)
        var acc = 0.0
        for ((a, b) in points.zipWithNext()) {
            val d = b - a
            val len = d.length()
            val t = if (len == 0.0) 0.0 else ((p - a).dot(d) / (len * len)).coerceIn(0.0, 1.0)
            val q = a + d * t
            val off = hypot(p.x - q.x, p.z - q.z)
            if (off < best.offset) best = Projection(acc + t * len, off)
            acc += len
        }
        return best
    }

    fun pointAt(s: Double): Vec2 {
        var acc = 0.0
        val segs = points.zipWithNext()
        for ((i, pair) in segs.withIndex()) {
            val (a, b) = pair
            val len = (b - a).length()
            if (s <= acc + len || i == segs.lastIndex) {
                val t = if (len == 0.0) 0.0 else ((s - acc) / len).coerceIn(0.0, 1.0)
                return a + (b - a) * t
            }
            acc += len
        }
        return points.last()
    }
}

sealed interface Guidance {
    data object Arrive : Guidance
    data class OffRoute(val offsetM: Double) : Guidance
    data object TurnAround : Guidance
    /** [relativeBearing] is radians relative to where the camera faces; positive means turn right. */
    data class Arrow(val relativeBearing: Double) : Guidance
}

class RouteFollower(
    private val route: Route,
    private val lookaheadM: Double = 4.0,
    private val offRouteM: Double = 3.0,
    private val arriveM: Double = 1.0,
    private val turnAroundDeg: Double = 120.0,
) {
    fun guide(pose: PlanPose): Guidance {
        val pr = route.project(pose.position)
        if (pr.offset > offRouteM) return Guidance.OffRoute(pr.offset)
        if (route.length - pr.distanceAlong < arriveM) return Guidance.Arrive
        val target = route.pointAt(minOf(route.length, pr.distanceAlong + lookaheadM))
        val b = normalizeAngle(headingOf(target.x - pose.position.x, target.z - pose.position.z) - pose.yaw)
        return if (abs(Math.toDegrees(b)) > turnAroundDeg) Guidance.TurnAround else Guidance.Arrow(b)
    }
}
