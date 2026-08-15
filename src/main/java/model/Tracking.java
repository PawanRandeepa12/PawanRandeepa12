package model;

import java.time.LocalDateTime;

public class Tracking {
    private String        trackingId;
    private String        shipmentId;
    private String        location;
    private LocalDateTime trackTime;
    private String        status;

    public Tracking(String trackingId, String shipmentId,
                    String location, LocalDateTime trackTime, String status) {
        this.trackingId = trackingId;
        this.shipmentId = shipmentId;
        this.location   = location;
        this.trackTime  = trackTime;
        this.status     = status;
    }

    public String        getTrackingId() { return trackingId; }
    public String        getShipmentId() { return shipmentId; }
    public String        getLocation()   { return location; }
    public LocalDateTime getTrackTime()  { return trackTime; }
    public String        getStatus()     { return status; }
}
