package dao;

import database.DBConnection;
import model.*;
import java.sql.*;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;

public class PaymentDAO {
    private Connection con = DBConnection.getConnection();

    public void addPayment(Payment p) {
        String sql = "INSERT INTO payments VALUES (?,?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, p.getPaymentId());
            ps.setString(2, p.getShipmentId());
            ps.setDouble(3, p.getAmount());
            ps.setDate(4, Date.valueOf(p.getPayDate()));
            ps.setString(5, p.getPayType());
            ps.setString(6, p.getStatus());
            ps.executeUpdate();
            System.out.println("Payment added.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public List<Payment> getAllPayments() {
        List<Payment> list = new ArrayList<>();
        String sql = "SELECT * FROM payments";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(makePayment(
                    rs.getString("payment_id"),
                    rs.getString("shipment_id"),
                    rs.getDouble("amount"),
                    rs.getDate("pay_date").toLocalDate(),
                    rs.getString("pay_type"),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public List<Payment> getByShipment(String shipmentId) {
        List<Payment> list = new ArrayList<>();
        String sql = "SELECT * FROM payments WHERE shipment_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, shipmentId);
            ResultSet rs = ps.executeQuery();
            while (rs.next()) {
                list.add(makePayment(
                    rs.getString("payment_id"),
                    rs.getString("shipment_id"),
                    rs.getDouble("amount"),
                    rs.getDate("pay_date").toLocalDate(),
                    rs.getString("pay_type"),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    private Payment makePayment(String id, String sid, double amt,
                                 LocalDate d, String payType, String status) {
        switch (payType) {
            case "CARD": return new CardPayment(id, sid, amt, d, status);
            case "ONLINE": return new OnlinePayment(id, sid, amt, d, status);
            default: return new CashPayment(id, sid, amt, d, status);
        }
    }

    public void updateStatus(String paymentId, String status) {
        String sql = "UPDATE payments SET status = ? WHERE payment_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, status);
            ps.setString(2, paymentId);
            ps.executeUpdate();
        } catch (SQLException e) { e.printStackTrace(); }
    }
}
