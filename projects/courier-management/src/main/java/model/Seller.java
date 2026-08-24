package model;

/**
 * A Seller (merchant partner) registered with the courier company.
 * Sellers hand over parcels for dispatch and earn a commission on each order.
 * Identity fields (id, name, phone) are inherited from Person.
 *
 * Module owner: Pawan Jayarathna
 */
public class Seller extends Person {

    private String  shopName;        // registered business / shop name
    private String  email;           // contact e-mail
    private String  pickupAddress;   // default parcel pickup location
    private double  commissionRate;  // commission percentage per order (5.0 = 5%)
    private int     ordersHandled;   // orders dispatched through the system
    private boolean active;          // account status

    public Seller(String id, String name, String phone) {
        this(id, name, phone, "", "", "", 0.0, 0, true);
    }

    public Seller(String id, String name, String phone, String shopName,
                  String email, String pickupAddress, double commissionRate,
                  int ordersHandled, boolean active) {
        super(id, name, phone);
        this.shopName       = shopName;
        this.email          = email;
        this.pickupAddress  = pickupAddress;
        this.commissionRate = commissionRate;
        this.ordersHandled  = ordersHandled;
        this.active         = active;
    }

    public String  getSellerId()       { return getId(); }
    public String  getShopName()       { return shopName; }
    public String  getEmail()          { return email; }
    public String  getPickupAddress()  { return pickupAddress; }
    public double  getCommissionRate() { return commissionRate; }
    public int     getOrdersHandled()  { return ordersHandled; }
    public boolean isActive()          { return active; }

    public void setShopName(String shopName)            { this.shopName       = shopName; }
    public void setEmail(String email)                  { this.email          = email; }
    public void setPickupAddress(String pickupAddress)  { this.pickupAddress  = pickupAddress; }
    public void setCommissionRate(double commissionRate){ this.commissionRate = commissionRate; }
    public void setOrdersHandled(int ordersHandled)     { this.ordersHandled  = ordersHandled; }
    public void setActive(boolean active)               { this.active         = active; }

    /** Commission (Rs.) the seller earns on an order of the given amount. */
    public double calculateCommission(double orderAmount) {
        return orderAmount * commissionRate / 100.0;
    }

    /** Registers one more handled order and returns the updated count. */
    public int recordOrder() {
        return ++ordersHandled;
    }

    /** One-line summary used by reports and notifications. */
    public String getSummary() {
        return String.format("[%s] %s (%s) — %s | %.1f%% commission | %d order(s) | %s",
                getRole(), getName(), getSellerId(),
                (shopName == null || shopName.isEmpty()) ? "No shop name" : shopName,
                commissionRate, ordersHandled, active ? "ACTIVE" : "INACTIVE");
    }

    @Override
    public String getRole() { return "SELLER"; }
}
