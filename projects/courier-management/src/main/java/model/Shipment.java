package model;

import java.time.LocalDate;

public class Shipment {
    private String    shipmentId;
    private String    customerId;
    private String    packageId;
    private String    agentId;
    private LocalDate shipDate;
    private LocalDate deliveryDate;
    private String    status;

    public Shipment(String shipmentId, String customerId, String packageId,
                    String agentId, LocalDate shipDate,
                    LocalDate deliveryDate, String status) {
        this.shipmentId   = shipmentId;
        this.customerId   = customerId;
        this.packageId    = packageId;
        this.agentId      = agentId;
        this.shipDate     = shipDate;
        this.deliveryDate = deliveryDate;
        this.status       = status;
    }

    public String    getShipmentId()   { return shipmentId; }
    public String    getCustomerId()   { return customerId; }
    public String    getPackageId()    { return packageId; }
    public String    getAgentId()      { return agentId; }
    public LocalDate getShipDate()     { return shipDate; }
    public LocalDate getDeliveryDate() { return deliveryDate; }
    public String    getStatus()       { return status; }

    public void setStatus(String status) { this.status = status; }
}
