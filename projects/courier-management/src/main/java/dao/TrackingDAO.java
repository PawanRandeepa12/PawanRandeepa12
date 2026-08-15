package dao;

import database.DBConnection;
import model.Tracking;
import java.sql.*;
import java.util.ArrayList;
import java.util.List;

public class TrackingDAO {
    private Connection con = DBConnection.getConnection();

    public void addTracking(Tracking t) {
        String sql = "INSERT INTO tracking VALUES (?,?,?,?,?)";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, t.getTrackingId());
            ps.setString(2, t.getShipmentId());
            ps.setString(3, t.getLocation());
            ps.setTimestamp(4, Timestamp.valueOf(t.getTrackTime()));
            ps.setString(5, t.getStatus());
            ps.executeUpdate();
            System.out.println("Tracking added.");
        } catch (SQLException e) { e.printStackTrace(); }
    }

    public List<Tracking> getByShipment(String shipmentId) {
        List<Tracking> list = new ArrayList<>();
        String sql = "SELECT * FROM tracking WHERE shipment_id = ? ORDER BY track_time DESC";
        try (PreparedStatement ps = con.prepareStatement(sql)) {
            ps.setString(1, shipmentId);
            ResultSet rs = ps.executeQuery();
            while (rs.next()) {
                list.add(new Tracking(
                    rs.getString("tracking_id"),
                    rs.getString("shipment_id"),
                    rs.getString("location"),
                    rs.getTimestamp("track_time").toLocalDateTime(),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }

    public List<Tracking> getAllTracking() {
        List<Tracking> list = new ArrayList<>();
        String sql = "SELECT * FROM tracking ORDER BY track_time DESC";
        try (Statement st = con.createStatement();
             ResultSet rs = st.executeQuery(sql)) {
            while (rs.next()) {
                list.add(new Tracking(
                    rs.getString("tracking_id"),
                    rs.getString("shipment_id"),
                    rs.getString("location"),
                    rs.getTimestamp("track_time").toLocalDateTime(),
                    rs.getString("status")
                ));
            }
        } catch (SQLException e) { e.printStackTrace(); }
        return list;
    }
}
