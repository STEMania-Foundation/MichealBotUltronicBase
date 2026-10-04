package org.firstinspires.ftc.teamcode.lib.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Telemetry that fans out to the Driver Station and to Panels.
 *
 * <p>Panels needs different handling from the Driver Station. TelemetryManager.update() clears its
 * pending line list on every call but only transmits when at least
 * TelemetryPluginConfig.telemetryUpdateInterval - 75 ms by default - has passed since the last
 * transmission. Pushing on every scheduler loop therefore builds a frame per loop and throws away
 * all but the one that happens to land after the interval expires, and whatever that surviving
 * frame is missing (a subsystem whose periodic() bailed out on that pass, say) is simply absent
 * from the panel. FTC telemetry does not work that way: it holds its items until it actually
 * transmits. That difference is why a reading can be on the Driver Station and not in Panels.
 *
 * <p>So entries accumulate here instead, keyed by caption so the newest value for each key wins
 * and the panel keeps a stable order, and the snapshot goes to Panels only when Panels is ready
 * to take it. Nothing is dropped before it has been sent.
 */
public class KTelemetry {

    /** One pending entry: a caption/value pair, or a bare line when {@code caption} is null. */
    private record Entry(String caption, String value) {
        String render() {
            return caption == null ? value : caption + ": " + value;
        }
    }

    private final Telemetry tele;
    private final TelemetryManager pTele;

    /** Caption -> entry. LinkedHashMap: re-putting a caption updates it in place, keeping order. */
    private final Map<String, Entry> entries = new LinkedHashMap<>();

    public KTelemetry(Telemetry tele, TelemetryManager pTele) {
        this.tele = tele;
        this.pTele = pTele;
    }

    public void addData(String name, Object data) {
        entries.put(name, new Entry(name, String.valueOf(data)));
    }

    public void addLine(String line) {
        entries.put(line, new Entry(null, line));
    }

    public void update(boolean active) {
        if (!active) {
            entries.clear();
            return;
        }

        for (Entry entry : entries.values()) {
            if (entry.caption() == null) {
                tele.addLine(entry.value());
            } else {
                tele.addData(entry.caption(), entry.value());
            }
        }
        tele.update();

        // Hand the snapshot over only when Panels will really transmit it, and clear only once it
        // has. addLine() rather than addData() because TelemetryManager.addData() would prefix the
        // caption a second time; render() already matches its "caption: value" format.
        if (pTele.getTimeSinceLastUpdate() >= pTele.getUpdateInterval()) {
            for (Entry entry : entries.values()) {
                pTele.addLine(entry.render());
            }
            pTele.update();
            entries.clear();
        }
    }
}
