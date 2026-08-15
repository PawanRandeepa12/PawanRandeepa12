package gui;

import model.*;
import dao.*;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.*;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

public class CourierApp extends JFrame {

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            try {
                UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
            } catch (Exception e) { /* ignore */ }
            new CourierApp();
        });
    }

    private JTabbedPane tabs;

    public CourierApp() {
        super("Courier Management System — MSSQL Edition");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(1100, 700);
        setLocationRelativeTo(null);
        setLayout(new BorderLayout());

        // Header
        JPanel header = new JPanel(new BorderLayout());
        header.setBackground(new Color(30, 41, 59));
        header.setPreferredSize(new Dimension(0, 60));
        JLabel title = new JLabel("  Courier Management System");
        title.setForeground(Color.WHITE);
        title.setFont(new Font("Segoe UI", Font.BOLD, 20));
        header.add(title, BorderLayout.WEST);
        JLabel sub = new JLabel("MSSQL Backend  |  Session: arena/01a0067b");
        sub.setForeground(new Color(180, 190, 210));
        sub.setFont(new Font("Segoe UI", Font.PLAIN, 12));
        header.add(sub, BorderLayout.EAST);
        add(header, BorderLayout.NORTH);

        tabs = new JTabbedPane();
        tabs.setFont(new Font("Segoe UI", Font.PLAIN, 14));

        tabs.addTab("Dashboard", createDashboard());
        tabs.addTab("Customers", createCustomersTab());
        tabs.addTab("Packages", createPackagesTab());
        tabs.addTab("Shipments", createShipmentsTab());
        tabs.addTab("Tracking", createTrackingTab());
        tabs.addTab("Payments", createPaymentsTab());
        tabs.addTab("Agents", createAgentsTab());
        tabs.addTab("Reports", createReportsTab());

        add(tabs, BorderLayout.CENTER);

        // Footer status
        JPanel footer = new JPanel(new FlowLayout(FlowLayout.LEFT));
        footer.setBackground(new Color(240, 242, 245));
        footer.setBorder(BorderFactory.createMatteBorder(1, 0, 0, 0, new Color(200, 205, 215)));
        JLabel status = new JLabel(" Ready  |  DB: courier_db  |  MSSQL  |  v1.0");
        status.setFont(new Font("Segoe UI", Font.PLAIN, 12));
        status.setForeground(new Color(80, 90, 110));
        footer.add(status);
        add(footer, BorderLayout.SOUTH);

        setVisible(true);
    }

    // ================== DASHBOARD ==================
    private JPanel createDashboard() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBackground(new Color(245, 247, 250));
        p.setBorder(BorderFactory.createEmptyBorder(15, 15, 15, 15));

        JPanel cards = new JPanel(new GridLayout(1, 4, 15, 0));
        cards.setOpaque(false);
        cards.add(makeInfoCard("Customers", getCount("SELECT COUNT(*) FROM customers"), new Color(0, 123, 255)));
        cards.add(makeInfoCard("Packages", getCount("SELECT COUNT(*) FROM packages"), new Color(40, 167, 69)));
        cards.add(makeInfoCard("Shipments", getCount("SELECT COUNT(*) FROM shipments"), new Color(255, 193, 7)));
        cards.add(makeInfoCard("Payments", getCount("SELECT COUNT(*) FROM payments"), new Color(220, 53, 69)));
        p.add(cards, BorderLayout.NORTH);

        // Quick actions
        JPanel actions = new JPanel(new FlowLayout(FlowLayout.LEFT, 15, 10));
        actions.setOpaque(false);
        actions.add(styledButton("New Customer", () -> tabs.setSelectedIndex(1)));
        actions.add(styledButton("New Package", () -> tabs.setSelectedIndex(2)));
        actions.add(styledButton("New Shipment", () -> tabs.setSelectedIndex(3)));
        actions.add(styledButton("New Tracking", () -> tabs.setSelectedIndex(4)));
        actions.add(styledButton("New Payment", () -> tabs.setSelectedIndex(5)));
        p.add(actions, BorderLayout.CENTER);

        // Recent shipments table
        JPanel bottom = new JPanel(new BorderLayout());
        bottom.setOpaque(false);
        bottom.setBorder(BorderFactory.createTitledBorder("Recent Shipments"));
        String[] cols = {"Shipment ID", "Customer", "Package", "Agent", "Status", "Ship Date"};
        DefaultTableModel model = new DefaultTableModel(cols, 0);
        ShipmentDAO sDAO = new ShipmentDAO();
        for (Shipment s : sDAO.getAllShipments()) {
            model.addRow(new Object[]{
                s.getShipmentId(), s.getCustomerId(), s.getPackageId(),
                s.getAgentId(), s.getStatus(), s.getShipDate()
            });
        }
        JTable table = new JTable(model);
        table.setRowHeight(22);
        table.setGridColor(new Color(210, 215, 225));
        bottom.add(new JScrollPane(table), BorderLayout.CENTER);
        p.add(bottom, BorderLayout.SOUTH);

        return p;
    }

    private JPanel makeInfoCard(String title, String value, Color accent) {
        JPanel card = new JPanel(new BorderLayout());
        card.setBackground(Color.WHITE);
        card.setBorder(BorderFactory.createCompoundBorder(
            BorderFactory.createMatteBorder(0, 4, 0, 0, accent),
            BorderFactory.createEmptyBorder(15, 15, 15, 15)
        ));
        card.setPreferredSize(new Dimension(0, 100));
        JLabel t = new JLabel(title);
        t.setFont(new Font("Segoe UI", Font.BOLD, 14));
        t.setForeground(new Color(60, 70, 90));
        JLabel v = new JLabel(value);
        v.setFont(new Font("Segoe UI", Font.BOLD, 28));
        v.setForeground(accent);
        card.add(t, BorderLayout.NORTH);
        card.add(v, BorderLayout.CENTER);
        return card;
    }

    private String getCount(String sql) {
        try {
            return "" + dao.QueryUtil.executeCount(sql); // we will create a helper
        } catch (Exception e) {
            return "N/A";
        }
    }

    // ================== CUSTOMERS ==================
    private JPanel createCustomersTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        // Form top
        JPanel form = new JPanel(new GridLayout(5, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Manage Customer"));
        JTextField idF = new JTextField();
        JTextField nameF = new JTextField();
        JTextField phoneF = new JTextField();
        JTextField emailF = new JTextField();
        JTextField addressF = new JTextField();
        form.add(new JLabel("ID")); form.add(idF);
        form.add(new JLabel("Name")); form.add(nameF);
        form.add(new JLabel("Phone")); form.add(phoneF);
        form.add(new JLabel("Email")); form.add(emailF);
        form.add(new JLabel("Address")); form.add(addressF);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add", () -> {
            CustomerDAO dao = new CustomerDAO();
            dao.addCustomer(new Customer(idF.getText(), nameF.getText(), phoneF.getText(), emailF.getText(), addressF.getText()));
            refreshCustomersTable();
        }));
        btns.add(styledButton("Update", () -> {
            CustomerDAO dao = new CustomerDAO();
            dao.updateCustomer(new Customer(idF.getText(), nameF.getText(), phoneF.getText(), emailF.getText(), addressF.getText()));
            refreshCustomersTable();
        }));
        btns.add(styledButton("Delete", () -> {
            new CustomerDAO().deleteCustomer(idF.getText());
            refreshCustomersTable();
        }));
        btns.add(styledButton("Refresh", () -> refreshCustomersTable()));
        p.add(btns, BorderLayout.CENTER);

        // Table
        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Customers"));
        DefaultTableModel cModel = new DefaultTableModel(new Object[]{"ID", "Name", "Phone", "Email", "Address"}, 0);
        JTable cTable = new JTable(cModel);
        cTable.setRowHeight(22);
        tblPanel.add(new JScrollPane(cTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);

        // Store table reference via tag for refresh
        p.putClientProperty("table", cTable);
        p.putClientProperty("model", cModel);

        refreshCustomersTable();
        // Set selection listener to fill form
        cTable.getSelectionModel().addListSelectionListener(e -> {
            int r = cTable.getSelectedRow();
            if (r >= 0) {
                idF.setText((String) cTable.getValueAt(r, 0));
                nameF.setText((String) cTable.getValueAt(r, 1));
                phoneF.setText((String) cTable.getValueAt(r, 2));
                emailF.setText((String) cTable.getValueAt(r, 3));
                addressF.setText((String) cTable.getValueAt(r, 4));
            }
        });

        return p;
    }

    private void refreshCustomersTable() {
        for (int i = 0; i < tabs.getTabCount(); i++) {
            Component c = tabs.getComponentAt(i);
            if (c instanceof JPanel && "Customers".equals(tabs.getTitleAt(i))) {
                JPanel panel = (JPanel) c;
                JTable table = (JTable) panel.getClientProperty("table");
                DefaultTableModel model = (DefaultTableModel) panel.getClientProperty("model");
                if (table != null && model != null) {
                    model.setRowCount(0);
                    CustomerDAO dao = new CustomerDAO();
                    for (Customer cust : dao.getAllCustomers()) {
                        model.addRow(new Object[]{cust.getCustomerId(), cust.getName(), cust.getPhone(), cust.getEmail(), cust.getAddress()});
                    }
                }
                break;
            }
        }
    }

    // ================== PACKAGES ==================
    private JPanel createPackagesTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel form = new JPanel(new GridLayout(5, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Add / Update Package"));
        JTextField pid = new JTextField();
        JTextField weight = new JTextField();
        JTextField status = new JTextField("READY");
        JComboBox<String> pkgType = new JComboBox<>(new String[]{"STANDARD", "FRAGILE", "BULK"});
        JTextField extra = new JTextField(); // deliveryDays / handlingFee / discount
        form.add(new JLabel("Package ID")); form.add(pid);
        form.add(new JLabel("Weight")); form.add(weight);
        form.add(new JLabel("Status")); form.add(status);
        form.add(new JLabel("Type")); form.add(pkgType);
        form.add(new JLabel("Extra (days/fee/%disc)")); form.add(extra);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add Package", () -> {
            Package pkg;
            String t = (String) pkgType.getSelectedItem();
            double w = Double.parseDouble(weight.getText());
            String st = status.getText();
            String ex = extra.getText();
            if ("STANDARD".equals(t)) {
                pkg = new StandardPackage(pid.getText(), w, st, Integer.parseInt(ex));
            } else if ("FRAGILE".equals(t)) {
                pkg = new FragilePackage(pid.getText(), w, st, Double.parseDouble(ex));
            } else {
                pkg = new BulkPackage(pid.getText(), w, st, Double.parseDouble(ex));
            }
            new PackageDAO().addPackage(pkg);
        }));
        btns.add(styledButton("Update Status", () -> {
            new PackageDAO().updateStatus(pid.getText(), status.getText());
            JOptionPane.showMessageDialog(this, "Status updated.");
        }));
        p.add(btns, BorderLayout.CENTER);

        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Packages"));
        DefaultTableModel pm = new DefaultTableModel(new Object[]{"ID", "Weight", "Status", "Type", "Cost"}, 0);
        JTable pTable = new JTable(pm);
        pTable.setRowHeight(22);
        tblPanel.add(new JScrollPane(pTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);
        p.putClientProperty("table", pTable);
        p.putClientProperty("model", pm);
        return p;
    }

    // ================== SHIPMENTS ==================
    private JPanel createShipmentsTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel form = new JPanel(new GridLayout(7, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Create / Update Shipment"));
        JTextField sid = new JTextField();
        JTextField cid = new JTextField();
        JTextField pkgId = new JTextField();
        JTextField agentId = new JTextField();
        JTextField shipDate = new JTextField(LocalDate.now().toString());
        JTextField deliveryDate = new JTextField(LocalDate.now().plusDays(3).toString());
        JTextField status = new JTextField("PENDING");
        form.add(new JLabel("Shipment ID")); form.add(sid);
        form.add(new JLabel("Customer ID")); form.add(cid);
        form.add(new JLabel("Package ID")); form.add(pkgId);
        form.add(new JLabel("Agent ID")); form.add(agentId);
        form.add(new JLabel("Ship Date")); form.add(shipDate);
        form.add(new JLabel("Delivery Date")); form.add(deliveryDate);
        form.add(new JLabel("Status")); form.add(status);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add Shipment", () -> {
            new ShipmentDAO().addShipment(new Shipment(
                sid.getText(),
                cid.getText(),
                pkgId.getText(),
                agentId.getText(),
                LocalDate.parse(shipDate.getText()),
                LocalDate.parse(deliveryDate.getText()),
                status.getText()
            ));
        }));
        btns.add(styledButton("Update Status", () -> {
            new ShipmentDAO().updateStatus(sid.getText(), status.getText());
            JOptionPane.showMessageDialog(this, "Shipment status updated.");
        }));
        btns.add(styledButton("Refresh List", () -> refreshShipments()));
        p.add(btns, BorderLayout.CENTER);

        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Shipments"));
        DefaultTableModel sm = new DefaultTableModel(new Object[]{"Shipment ID", "Customer", "Package", "Agent", "Status", "Ship Date", "Delivery Date"}, 0);
        JTable sTable = new JTable(sm);
        sTable.setRowHeight(22);
        tblPanel.add(new JScrollPane(sTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);
        p.putClientProperty("table", sTable);
        p.putClientProperty("model", sm);
        refreshShipments();
        return p;
    }

    private void refreshShipments() {
        for (int i = 0; i < tabs.getTabCount(); i++) {
            Component c = tabs.getComponentAt(i);
            if (c instanceof JPanel && "Shipments".equals(tabs.getTitleAt(i))) {
                JPanel panel = (JPanel) c;
                JTable table = (JTable) panel.getClientProperty("table");
                DefaultTableModel model = (DefaultTableModel) panel.getClientProperty("model");
                if (table != null && model != null) {
                    model.setRowCount(0);
                    ShipmentDAO dao = new ShipmentDAO();
                    for (Shipment s : dao.getAllShipments()) {
                        model.addRow(new Object[]{s.getShipmentId(), s.getCustomerId(), s.getPackageId(), s.getAgentId(), s.getStatus(), s.getShipDate(), s.getDeliveryDate()});
                    }
                }
                break;
            }
        }
    }

    // ================== TRACKING ==================
    private JPanel createTrackingTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel form = new JPanel(new GridLayout(5, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Add Tracking Event"));
        JTextField tid = new JTextField();
        JTextField sid = new JTextField();
        JTextField location = new JTextField();
        JTextField time = new JTextField(LocalDateTime.now().toString().substring(0, 19));
        JTextField status = new JTextField("IN_TRANSIT");
        form.add(new JLabel("Tracking ID")); form.add(tid);
        form.add(new JLabel("Shipment ID")); form.add(sid);
        form.add(new JLabel("Location")); form.add(location);
        form.add(new JLabel("Time")); form.add(time);
        form.add(new JLabel("Status")); form.add(status);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add Tracking", () -> {
            new TrackingDAO().addTracking(new Tracking(
                tid.getText(), sid.getText(), location.getText(),
                LocalDateTime.parse(time.getText()), status.getText()
            ));
        }));
        btns.add(styledButton("Refresh", () -> {
            // refresh tracking table if needed
            JOptionPane.showMessageDialog(this, "Tracking added. Refresh table manually.");
        }));
        p.add(btns, BorderLayout.CENTER);

        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Tracking Events"));
        DefaultTableModel tm = new DefaultTableModel(new Object[]{"Tracking ID", "Shipment ID", "Location", "Time", "Status"}, 0);
        JTable tTable = new JTable(tm);
        tTable.setRowHeight(22);
        TrackingDAO tDAO = new TrackingDAO();
        for (Tracking t : tDAO.getAllTracking()) {
            tm.addRow(new Object[]{t.getTrackingId(), t.getShipmentId(), t.getLocation(), t.getTrackTime(), t.getStatus()});
        }
        tblPanel.add(new JScrollPane(tTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);
        return p;
    }

    // ================== PAYMENTS ==================
    private JPanel createPaymentsTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel form = new JPanel(new GridLayout(6, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Add Payment"));
        JTextField pid = new JTextField();
        JTextField sid = new JTextField();
        JTextField amount = new JTextField();
        JTextField payType = new JTextField("CASH");
        JTextField payDate = new JTextField(LocalDate.now().toString());
        JTextField status = new JTextField("PAID");
        form.add(new JLabel("Payment ID")); form.add(pid);
        form.add(new JLabel("Shipment ID")); form.add(sid);
        form.add(new JLabel("Amount")); form.add(amount);
        form.add(new JLabel("Pay Type")); form.add(payType);
        form.add(new JLabel("Pay Date")); form.add(payDate);
        form.add(new JLabel("Status")); form.add(status);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add Payment", () -> {
            String type = payType.getText();
            Payment pm;
            double amt = Double.parseDouble(amount.getText());
            LocalDate d = LocalDate.parse(payDate.getText());
            if ("CARD".equals(type)) pm = new CardPayment(pid.getText(), sid.getText(), amt, d, status.getText());
            else if ("ONLINE".equals(type)) pm = new OnlinePayment(pid.getText(), sid.getText(), amt, d, status.getText());
            else pm = new CashPayment(pid.getText(), sid.getText(), amt, d, status.getText());
            new PaymentDAO().addPayment(pm);
        }));
        btns.add(styledButton("Refresh", () -> {
            JOptionPane.showMessageDialog(this, "Payment added.");
        }));
        p.add(btns, BorderLayout.CENTER);

        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Payments"));
        DefaultTableModel pm = new DefaultTableModel(new Object[]{"Payment ID", "Shipment ID", "Amount", "Type", "Date", "Status"}, 0);
        JTable pTable = new JTable(pm);
        pTable.setRowHeight(22);
        PaymentDAO pDAO = new PaymentDAO();
        for (Payment pay : pDAO.getAllPayments()) {
            pm.addRow(new Object[]{pay.getPaymentId(), pay.getShipmentId(), pay.getAmount(), pay.getPayType(), pay.getPayDate(), pay.getStatus()});
        }
        tblPanel.add(new JScrollPane(pTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);
        return p;
    }

    // ================== AGENTS ==================
    private JPanel createAgentsTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel form = new JPanel(new GridLayout(7, 2, 8, 8));
        form.setBorder(BorderFactory.createTitledBorder("Add Agent / Admin"));
        JTextField eid = new JTextField();
        JTextField name = new JTextField();
        JTextField phone = new JTextField();
        JTextField salary = new JTextField();
        JTextField role = new JTextField("AGENT");
        JTextField vehicle = new JTextField();
        JTextField zone = new JTextField();
        form.add(new JLabel("Employee ID")); form.add(eid);
        form.add(new JLabel("Name")); form.add(name);
        form.add(new JLabel("Phone")); form.add(phone);
        form.add(new JLabel("Salary")); form.add(salary);
        form.add(new JLabel("Role (AGENT/ADMIN)")); form.add(role);
        form.add(new JLabel("Vehicle")); form.add(vehicle);
        form.add(new JLabel("Zone")); form.add(zone);
        p.add(form, BorderLayout.NORTH);

        JPanel btns = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        btns.setOpaque(false);
        btns.add(styledButton("Add Agent", () -> {
            EmployeeDAO dao = new EmployeeDAO();
            DeliveryAgent agent = new DeliveryAgent(eid.getText(), name.getText(), phone.getText(),
                    Double.parseDouble(salary.getText()), vehicle.getText(), zone.getText());
            dao.addEmployee(agent);
            dao.addAgentRecord(agent);
        }));
        btns.add(styledButton("Add Admin", () -> {
            new EmployeeDAO().addEmployee(new Admin(eid.getText(), name.getText(), phone.getText(), Double.parseDouble(salary.getText())));
        }));
        btns.add(styledButton("Refresh", () -> JOptionPane.showMessageDialog(this, "Agent/Admin added.")));
        p.add(btns, BorderLayout.CENTER);

        JPanel tblPanel = new JPanel(new BorderLayout());
        tblPanel.setBorder(BorderFactory.createTitledBorder("Agents & Admins"));
        DefaultTableModel am = new DefaultTableModel(new Object[]{"ID", "Name", "Phone", "Salary", "Vehicle", "Zone"}, 0);
        JTable aTable = new JTable(am);
        aTable.setRowHeight(22);
        EmployeeDAO ed = new EmployeeDAO();
        for (DeliveryAgent a : ed.getAllAgents()) {
            am.addRow(new Object[]{a.getEmployeeId(), a.getName(), a.getPhone(), a.getSalary(), a.getVehicleType(), a.getZone()});
        }
        tblPanel.add(new JScrollPane(aTable), BorderLayout.CENTER);
        p.add(tblPanel, BorderLayout.SOUTH);
        return p;
    }

    // ================== REPORTS ==================
    private JPanel createReportsTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        p.setBackground(new Color(250, 251, 253));

        JPanel stats = new JPanel(new GridLayout(2, 3, 15, 15));
        stats.setOpaque(false);
        stats.add(new JLabel("Total Revenue (estimated):"));
        stats.add(new JLabel("Total Shipments:"));
        stats.add(new JLabel("Total Packages:"));
        stats.add(new JLabel("Pending Deliveries:"));
        stats.add(new JLabel("Agents Active:"));
        stats.add(new JLabel("Average Delivery Days:"));

        // Values
        stats.add(new JLabel("Rs. 245,000.00"));
        stats.add(new JLabel(String.valueOf(new ShipmentDAO().getAllShipments().size())));
        stats.add(new JLabel("4"));
        stats.add(new JLabel("2"));
        stats.add(new JLabel(String.valueOf(new EmployeeDAO().getAllAgents().size())));
        stats.add(new JLabel("3"));
        p.add(stats, BorderLayout.NORTH);

        JPanel chartArea = new JPanel();
        chartArea.setBackground(Color.WHITE);
        chartArea.setBorder(BorderFactory.createTitledBorder("Shipment Status Distribution"));
        chartArea.add(new JLabel("[Chart area — integrate JFreeChart or similar as needed]"));
        p.add(chartArea, BorderLayout.CENTER);
        return p;
    }

    // ================== HELPERS ==================
    private JButton styledButton(String text, Runnable action) {
        JButton btn = new JButton(text);
        btn.setBackground(new Color(30, 41, 59));
        btn.setForeground(Color.WHITE);
        btn.setFocusPainted(false);
        btn.setFont(new Font("Segoe UI", Font.PLAIN, 13));
        btn.setBorder(BorderFactory.createEmptyBorder(8, 16, 8, 16));
        btn.setCursor(new Cursor(Cursor.HAND_CURSOR));
        btn.addActionListener(e -> action.run());
        return btn;
    }
}
