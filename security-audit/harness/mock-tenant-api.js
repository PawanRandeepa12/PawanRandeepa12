// Mock multi-tenant API — LOCAL TRAINING TARGET ONLY. Never point at real systems.
// Run:  VULN=0 node mock-tenant-api.js   (safe: tenant checks on  -> harness should PASS)
//       VULN=1 node mock-tenant-api.js   (vulnerable: checks off -> harness should FAIL loudly)
// Sessions are passed via x-test-session header: ownerA | memberA | ownerB | shopperA | (absent=anon)
const http = require('node:http');
const VULN = process.env.VULN === '1';

const SESSIONS = {
  ownerA:   { org: 'orgA', role: 'owner' },
  memberA:  { org: 'orgA', role: 'member' },
  ownerB:   { org: 'orgB', role: 'owner' },
  shopperA: { org: 'orgA', role: 'shopper' },
};
const ORDERS = {
  'ord-A1': { id: 'ord-A1', org: 'orgA', canary: 'CANARY-A-cx7q2', total: 9999, note: '' },
  'ord-B1': { id: 'ord-B1', org: 'orgB', canary: 'CANARY-B-cx7q2', total: 4999, note: '' },
};
const NOT_FOUND = JSON.stringify({ error: 'not_found' });
const DENIED = JSON.stringify({ error: 'unauthorized' });

function send(res, code, body) {
  res.writeHead(code, { 'content-type': 'application/json' });
  res.end(body);
}

const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://mock');
  const sess = SESSIONS[req.headers['x-test-session']] || null;

  // --- Admin area: separate platform role (nobody in this mock has it) ---
  if (url.pathname === '/api/admin/users') {
    if (!sess) return send(res, 401, DENIED);
    if (VULN) return send(res, 200, JSON.stringify({ users: ['ownerA', 'ownerB'] })); // vuln: no role check
    return send(res, 404, NOT_FOUND); // safe: neutral, no admin oracle
  }

  // --- Order search / filter (IDOR-T04: org_id must come from session, never query) ---
  if (url.pathname === '/api/orders' && req.method === 'GET') {
    if (!sess) return send(res, 401, DENIED);
    if (sess.role === 'shopper') return send(res, 404, NOT_FOUND);
    const requestedOrg = url.searchParams.get('org_id');
    const effectiveOrg = VULN && requestedOrg ? requestedOrg : sess.org; // vuln honors query param
    const rows = Object.values(ORDERS).filter((o) => o.org === effectiveOrg);
    return send(res, 200, JSON.stringify({ orders: rows }));
  }

  // --- Order detail (IDOR-R01) + note write (IDOR-W02, non-destructive canary field) ---
  const m = url.pathname.match(/^\/api\/orders\/([^/]+)(\/note)?$/);
  if (m) {
    if (!sess) return send(res, 401, DENIED);
    if (sess.role === 'shopper') return send(res, 404, NOT_FOUND);
    const order = ORDERS[m[1]];
    if (!order) return send(res, 404, NOT_FOUND);
    if (!VULN && order.org !== sess.org) return send(res, 404, NOT_FOUND); // safe: neutral 404
    if (m[2] && req.method === 'PUT') {
      if (!VULN && sess.role !== 'owner') return send(res, 403, DENIED); // safe: owner-only write
      let body = '';
      req.on('data', (c) => (body += c));
      return req.on('end', () => {
        try { order.note = JSON.parse(body).note || ''; } catch { /* ignore */ }
        return send(res, 200, JSON.stringify({ ok: true, order }));
      });
    }
    if (!m[2] && req.method === 'GET') return send(res, 200, JSON.stringify({ order }));
    return send(res, 405, NOT_FOUND);
  }

  return send(res, 404, NOT_FOUND);
});

server.listen(8901, '127.0.0.1', () =>
  console.log(`mock-tenant-api listening on 127.0.0.1:8901 (VULN=${VULN ? 'ON — checks disabled' : 'OFF — checks enabled'})`)
);
