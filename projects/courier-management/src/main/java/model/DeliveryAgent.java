package model;

public class DeliveryAgent extends Employee {
    private String vehicleType;
    private String zone;

    public DeliveryAgent(String id, String name, String phone,
                         double salary, String vehicleType, String zone) {
        super(id, name, phone, salary);
        this.vehicleType = vehicleType;
        this.zone        = zone;
    }

    public String getVehicleType() { return vehicleType; }
    public String getZone()        { return zone; }

    public void setZone(String zone) { this.zone = zone; }

    @Override
    public String getRole() { return "AGENT"; }
}
