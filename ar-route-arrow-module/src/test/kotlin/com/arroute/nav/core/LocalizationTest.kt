package com.arroute.nav.core

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class LocalizationTest {
    private fun marker(id: String, status: MarkerStatus = MarkerStatus.SURVEYED, floor: String = "F1", ver: String = "v1") =
        MarkerRecord(id, floor, ver, 5.0, 5.0, 1.2, PI / 2, Mount.WALL, status, 0.02)

    private fun mgr(vararg m: MarkerRecord, dist: Double = 20.0, secs: Double = 90.0) =
        LocalizationManager(m.toList(), "v1", AnchorBudget(dist, secs))

    /** Marker seen at session (0, 0) with session reference yaw 0. */
    private fun seen(id: String = "M1", full: Boolean = true) = listOf(SessionObservation(id, 0.0, 1.2, 0.0, 0.0, full))

    @Test fun registrationMapsCameraIntoPlan() {
        val m = mgr(marker("M1"))
        assertTrue(m.onMarkerObserved(seen(), true, 0.0) is ScanResult.Accepted)
        // Marker: session yaw 0 -> plan yaw PI/2, so rotation is +PI/2. Camera 1 m along session +x -> plan (5, 6).
        val p = m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 1.0)!!
        assertEquals(5.0, p.position.x, 1e-9)
        assertEquals(6.0, p.position.z, 1e-9)
        assertEquals(PI / 2, p.yaw, 1e-9)
        assertEquals("F1", p.floorId)
    }

    @Test fun floorHeightComesFromMarkerHeight() {
        val m = mgr(marker("M1"))
        m.onMarkerObserved(seen(), true, 0.0)
        assertEquals(0.0, m.floorYSession!!, 1e-9)
    }

    @Test fun unsurveyedMarkerCannotLocalize() {
        for (st in listOf(MarkerStatus.UNSURVEYED, MarkerStatus.SURVEY_PENDING, MarkerStatus.SURVEY_FAILED, MarkerStatus.STALE)) {
            val m = mgr(marker("M1", st))
            val r = m.onMarkerObserved(seen(), true, 0.0) as ScanResult.Rejected
            assertEquals(RejectReason.NOT_SURVEYED, r.reason)
            assertTrue(m.state is LocState.NoFix)
        }
    }

    @Test fun rejectsBadScans() {
        val m = mgr(marker("M1"), marker("M2"), marker("OLD", ver = "v0"))
        fun why(o: List<SessionObservation>, tn: Boolean = true) = (m.onMarkerObserved(o, tn, 0.0) as ScanResult.Rejected).reason
        assertEquals(RejectReason.TRACKING_NOT_NORMAL, why(seen(), false))
        assertEquals(RejectReason.NOT_FULLY_TRACKED, why(seen(full = false)))
        assertEquals(RejectReason.AMBIGUOUS_MULTIPLE_MARKERS, why(seen("M1") + seen("M2")))
        assertEquals(RejectReason.UNKNOWN_MARKER, why(seen("ZZ")))
        assertEquals(RejectReason.WRONG_MAP_VERSION, why(seen("OLD")))
        assertTrue(m.state is LocState.NoFix)
    }

    @Test fun trackingLossNeverReusesOldPose() {
        val m = mgr(marker("M1"))
        m.onMarkerObserved(seen(), true, 0.0)
        assertNotNull(m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 1.0))
        assertNull(m.onFrame(false, 1.0, 0.0, 1.0, 0.0, 2.0))
        assertTrue(m.state is LocState.Lost)
        assertNull(m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 3.0)) // tracking is back, pose must stay gone
        assertNull(m.floorYSession)
        assertTrue(m.onMarkerObserved(seen(), true, 4.0) is ScanResult.Accepted)
        assertNotNull(m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 5.0))
    }

    @Test fun sessionRestartFloorChangeMapChangeAndNoCameraZoneInvalidate() {
        val triggers = listOf<(LocalizationManager) -> Unit>(
            { it.onSessionRestart() }, { it.onFloorChange("F2") }, { it.onMapVersionChange("v2") }, { it.setCameraAllowed(false) },
        )
        for (t in triggers) {
            val m = mgr(marker("M1"))
            m.onMarkerObserved(seen(), true, 0.0)
            t(m)
            assertNull(m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 1.0))
            assertTrue(m.state !is LocState.Fine)
        }
    }

    @Test fun markerOnWrongFloorDropsFineAndAsksUser() {
        val m = mgr(marker("M1", floor = "F1"), marker("M9", floor = "F9"))
        m.onMarkerObserved(seen("M1"), true, 0.0)
        m.expectedFloorId = "F1"
        val r = m.onMarkerObserved(seen("M9"), true, 1.0) as ScanResult.Rejected
        assertEquals(RejectReason.FLOOR_MISMATCH, r.reason)
        assertTrue(m.state is LocState.Coarse)
        assertNull(m.onFrame(true, 1.0, 0.0, 1.0, 0.0, 2.0))
    }

    @Test fun distanceAndTimeBudgetsDropToCoarse() {
        val a = mgr(marker("M1"), dist = 5.0)
        a.onMarkerObserved(seen(), true, 0.0)
        var x = 0.0; var out: PlanPose? = null
        repeat(8) { x += 1.0; out = a.onFrame(true, x, 0.0, 1.0, 0.0, x) }
        assertNull(out); assertTrue(a.state is LocState.Coarse)
        val b = mgr(marker("M1"), secs = 10.0)
        b.onMarkerObserved(seen(), true, 0.0)
        assertNotNull(b.onFrame(true, 0.5, 0.0, 1.0, 0.0, 9.0))
        assertNull(b.onFrame(true, 0.6, 0.0, 1.0, 0.0, 11.0))
        assertTrue(b.state is LocState.Coarse)
    }

    @Test fun poseJumpIsTreatedAsTrackingFault() {
        val m = mgr(marker("M1"))
        m.onMarkerObserved(seen(), true, 0.0)
        m.onFrame(true, 0.0, 0.0, 1.0, 0.0, 0.1)
        assertNull(m.onFrame(true, 10.0, 0.0, 1.0, 0.0, 0.2))
        assertTrue(m.state is LocState.Lost)
    }
}
