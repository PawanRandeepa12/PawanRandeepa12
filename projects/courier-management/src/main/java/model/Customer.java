package model;

public class Customer extends Person {
    private String email;
    private String address;

    public Customer(String id, String name, String phone,
                    String email, String address) {
        super(id, name, phone);
        this.email   = email;
        this.address = address;
    }

    public String getCustomerId() { return getId(); }
    public String getEmail()      { return email; }
    public String getAddress()    { return address; }

    public void setEmail(String email)     { this.email   = email; }
    public void setAddress(String address) { this.address = address; }

    @Override
    public String getRole() { return "CUSTOMER"; }
}
