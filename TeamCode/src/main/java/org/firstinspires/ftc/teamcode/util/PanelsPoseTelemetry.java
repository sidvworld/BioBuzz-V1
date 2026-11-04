package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;

/** Displays the current Pedro pose in Panels without changing Driver Station telemetry. */
public final class PanelsPoseTelemetry {
    private static final double ROBOT_RADIUS_INCHES = 9.0;

    private PanelsPoseTelemetry() {}

    public static void update(Pose pose) {
        FieldManager field = PanelsField.INSTANCE.getField();
        // Pedro uses inches from the field corner and headings in radians.
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        field.setStyle("transparent", "#00bcd4", 0.75);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS_INCHES);
        field.line(
                pose.x() + ROBOT_RADIUS_INCHES * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS_INCHES * Math.sin(pose.heading())
        );
        field.update();

        TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Robot X (in)", pose.x());
        telemetry.addData("Robot Y (in)", pose.y());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
