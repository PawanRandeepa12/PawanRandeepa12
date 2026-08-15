package model;

import java.time.LocalDate;

public abstract class Payment {
    private String    paymentId;
    private String    shipmentId;
    private double    amount;
    private LocalDate payDate;
    private String    status;

    public Payment(String paymentId, String shipmentId,
                   double amount, LocalDate payDate, String status) {
        this.paymentId  = paymentId;
        this.shipmentId = shipmentId;
        this.amount     = amount;
        this.payDate    = payDate;
        this.status     = status;
    }

    public String    getPaymentId()  { return paymentId; }
    public String    getShipmentId() { return shipmentId; }
    public double    getAmount()     { return amount; }
    public LocalDate getPayDate()    { return payDate; }
    public String    getStatus()     { return status; }

    public abstract boolean processPayment();
    public abstract String  getPayType();
}
