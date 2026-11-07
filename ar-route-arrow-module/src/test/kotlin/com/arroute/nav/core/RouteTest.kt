package com.arroute.nav.core

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class RouteTest {
    private val route = Route(listOf(Vec2(2.0, 1.0), Vec2(2.0, 9.0), Vec2(9.0, 9.0)))
    private val f = RouteFollower(route)
    private fun pose(x: Double, z: Double, yaw: Double) = PlanPose(Vec2(x, z), yaw, "F1", "v1")

    @Test fun straightAheadWhenFacingRoute() {
        val g = f.guide(pose(2.0, 2.0, PI / 2)) as Guidance.Arrow
        assertEquals(0.0, g.relativeBearing, 1e-9)
    }

    @Test fun rightTurnIsPositiveBearing() {
        val g = f.guide(pose(2.0, 2.0, 0.0)) as Guidance.Arrow // facing +x, route goes +z
        assertEquals(PI / 2, g.relativeBearing, 1e-9)
    }

    @Test fun turnAroundWhenFacingAway() {
        assertEquals(Guidance.TurnAround, f.guide(pose(2.0, 2.0, -PI / 2)))
    }

    @Test fun offRouteAndArrive() {
        assertTrue(f.guide(pose(8.0, 2.0, 0.0)) is Guidance.OffRoute)
        assertEquals(Guidance.Arrive, f.guide(pose(8.5, 9.0, 0.0)))
    }

    @Test fun lookaheadFollowsCorner() {
        // 2 m before the corner the 4 m lookahead target is 2 m past it, so the arrow bends toward +x.
        val g = f.guide(pose(2.0, 7.0, PI / 2)) as Guidance.Arrow
        assertTrue(g.relativeBearing < 0.0) // facing +z, target is at larger x: that is a LEFT turn here
    }

    @Test fun arrowLiesOnFloorAhead() {
        val v = ArrowGeometry.vertices(0.0, 0.0, 1.0, 0.0, floorY = -0.2, relativeBearing = 0.0)
        assertTrue(v.all { it.y == -0.2 })
        assertEquals(2.4 + 0.9, v[0].x, 1e-9)
        assertEquals(0.0, v[0].z, 1e-9)
        // arrow turned right by 90 degrees points toward +z when the camera faces +x
        val r = ArrowGeometry.vertices(0.0, 0.0, 1.0, 0.0, 0.0, PI / 2)
        assertEquals(2.4, r[0].x, 1e-9)
        assertEquals(0.9, r[0].z, 1e-9)
    }
}
