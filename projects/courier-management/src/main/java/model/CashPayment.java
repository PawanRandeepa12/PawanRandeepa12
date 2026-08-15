package model;

import java.time.LocalDate;

public class CashPayment extends Payment {

    public CashPayment(String paymentId, String shipmentId,
                       double amount, LocalDate payDate, String status) {
        super(paymentId, shipmentId, amount, payDate, status);
    }

    @Override
    public boolean processPayment() {
        System.out.println("Cash payment of Rs." + getAmount() + " processed.");
        return true;
    }

    @Override
    public String getPayType() { return "CASH"; }
}
