package dao;

import database.DBConnection;
import model.Shipment;
import java.sql.*;
import java.util.ArrayList;
import java.util.List;

public class ShipmentDAO {

    private Connection con = DBConnection.getConnection();

    public void addShipment(Shipment s) {
        String sql = "INSERT INTO shipments VALUES (?,?,?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, s.getShipmentId());
            ps.setString(2, s.getCustomerId());
            ps.setString(3, s.getPackageId());
            ps.setString(4, s.getAgentId());
            ps.setDate  (5, Date.valueOf(s.getShipDate()));
            ps.setDate  (6, Date.valueOf(s.getDeliveryDate()));
            ps.setString(7, s.getStatus());
            ps.executeUpdate();
            System.out.println("Shipment created: " + s.getShipmentId());
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public void updateStatus(String shipmentId, String status) {
        String sql = "UPDATE shipments SET status=? WHERE shipment_id=?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, status);
            ps.setString(2, shipmentId);
            ps.executeUpdate();
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public Shipment getShipment(String shipmentId) {
        String sql = "SELECT * FROM shipments WHERE shipment_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, shipmentId);
            ResultSet rs = ps.executeQuery();
            if (rs.next()) {
                return new Shipment(
                    rs.getString("shipment_id"),
                    rs.getString("customer_id"),
                    rs.getString("package_id"),
                    rs.getString("agent_id"),
                    rs.getDate("ship_date").toLocalDate(),
                    rs.getDate("delivery_date").toLocalDate(),
                    rs.getString("status")
                );
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return null;
    }

    public List<Shipment> getByCustomer(String customerId) {
        List<Shipment> list = new ArrayList<>();
        String sql = "SELECT * FROM shipments WHERE customer_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, customerId);
            ResultSet rs = ps.executeQuery();
            while (rs.next()) {
                list.add(new Shipment(
                    rs.getString("shipment_id"),
                    rs.getString("customer_id"),
                    rs.getString("package_id"),
                    rs.getString("agent_id"),
                    rs.getDate("ship_date").toLocalDate(),
                    rs.getDate("delivery_date").toLocalDate(),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public List<Shipment> getAllShipments() {
        List<Shipment> list = new ArrayList<>();
        String sql = "SELECT * FROM shipments";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(new Shipment(
                    rs.getString("shipment_id"),
                    rs.getString("customer_id"),
                    rs.getString("package_id"),
                    rs.getString("agent_id"),
                    rs.getDate("ship_date").toLocalDate(),
                    rs.getDate("delivery_date").toLocalDate(),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public void deleteShipment(String id) {
        String sql = "DELETE FROM shipments WHERE shipment_id = ?";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, id);
            ps.executeUpdate();
        } catch (SQLException e) { e.printStackTrace(); }
    }
}
