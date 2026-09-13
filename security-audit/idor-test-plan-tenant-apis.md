# IDOR / Tenant-Isolation Test Plan — Storzy Tenant APIs
### Companion to `storzy-iconnix-security-assessment-2026-09-13.md` (S-01, S-07, S-13, S-16)

| Field | Detail |
|---|---|
| **Objective** | Prove (or disprove) that every tenant-scoped object and action is bound to the caller's organization + role, server-side, with neutral failure responses. |
| **Method** | Authorized functional access-control testing using **accounts and tenants you own**. No exploit code, no mass enumeration, no third-party data access. |
| **Standards** | OWASP Top 10 A01:2021 · WSTG Authorization Testing (OTG-AUTHZ-001/002/004) · ASVS V4.1/V4.2, V13.4 · CWE-639, CWE-285, CWE-862 |
| **Date / version** | 2026-09-13 · v1.0 |

---

## 0. Authorization gate (do not skip)

- [ ] **Written authorization** from Storzy/ICONNIX naming: scope (staging preferred; prod only if explicitly allowed), test tenant IDs, allowed techniques, rate limits, test window, emergency contact, and data-handling rules.
- [ ] Storzy Terms §04 forbids accessing another merchant's data — your authorization letter is what makes §B testing legal. Keep it with the report.
- [ ] **Forbidden even when authorized:** touching real merchants' tenants, storing/copying third-party PII, credential attacks on others' accounts, load/DoS testing, and any technique outside the letter.
- [ ] **Accidental-exposure rule:** if any response ever contains data that is not your own canary data — **stop that test immediately, do not save/paginate/screenshot the payload, note only the request metadata (endpoint, time, object ID pattern), and report to the owner.** Redact aggressively.

## 1. Test environment setup

### 1.1 Accounts & tenants (all created by you, the validator)

| Alias | Role | Purpose |
|---|---|---|
| Owner-A / Store-A | Merchant owner, tenant A | Baseline victim tenant |
| Member-A | Invited workspace member, tenant A | Horizontal role + vertical escalation tests |
| Owner-B / Store-B | Merchant owner, tenant B | Attacker tenant (attempts A-access) |
| Shopper-S | Shopper account on Store-A (and one on Store-B) | Shopper↔merchant boundary tests |
| Anon | Logged out | Unauthenticated baseline |

Use distinct emails per alias (e.g. `sec-owner-a@<your-test-domain>`). Enable MFA on Owner-A if available, to also cover step-up paths.

### 1.2 Canary records (planted only in YOUR tenants)

Create uniquely identifiable records so any cross-tenant return is unambiguous without touching real data:

- Store-A: product `CANARY-A-<random>` (price LKR 9,999, stock 1, draft/unpublished if possible), order `CANARY-ORDER-A`, customer `Canary A <canary-a+…>`, review draft, one uploaded file, one gateway-settings read (do not enter real gateway secrets — use sandbox/dummy values).
- Store-B: mirror set (`CANARY-B-…`).
- Naming rule: random suffix per run (e.g. `cx7q2`) so findings can't be confused with other testers' data.

### 1.3 Tooling baseline

- Intercepting proxy (Burp/ZAP) with project scope limited to in-scope hosts; TLS trust installed for your own browser only.
- Two isolated browser profiles (or containers): Profile-A (Owner-A/Member-A/Shopper-S-A), Profile-B (Owner-B) — prevents session mix-ups that cause false positives.
- Request log: Burp project file + exported evidence pack (see §7).

---

## 2. Phase 1 — Endpoint & object inventory (passive, your tenants only)

Goal: a complete route × object × method map before a single ID is swapped.

1. Crawl *your own* sessions: sign in as each alias, click through dashboard/storefront, and record every `/api/*`, Server Action call, and RSC fetch (Burp history / DevTools Network).
2. Harvest client sources passively: list fetch URLs + route handlers in JS bundles served to you; confirm `robots.txt` prefixes (`/s/ /dashboard/ /admin/ /api/ /template/ /invite/`) resolve to *your* tenant context.
3. Record the **identifier scheme per object**: numeric sequence? UUIDv4? slug? short-hash? Note which are guessable (numeric/short/slugs) vs unguessable (UUID) — guessable IDs escalate any missing check from "needs leak" to "directly enumerable."
4. Note nested routes (`/orders/<id>/items/<itemId>`), bulk endpoints (arrays of IDs), and filter params (`org_id`, `store_id`, `store`, `handle`, `owner`, `account`).
5. Output: **route inventory table** (template in §7.2). Every row becomes test rows in §3–§5.

## 3. Phase 2 — Session baseline (prove your harness works)

| ID | Test | Steps (own accounts) | Expected (pass) |
|---|---|---|---|
| BASE-01 | Authenticated self-access | Each alias reads/updates its *own* canary objects via UI + replayed API call | 200 with own data; identical via UI and replay |
| BASE-02 | Anonymous denied | Replay the same calls with no cookies | 401/redirect-to-login, no data |
| BASE-03 | Wrong-password / bad-token | Tamper one byte of session cookie | Treated as anonymous (no partial data, no 500 trace) |
| BASE-04 | Role reflection | Record `role`/`org_id` the server *acts on* (not what the client sends) | Server derives both from session; client-sent role/org fields ignored |

Only proceed when BASE-01–04 behave — otherwise you can't distinguish IDOR from broken harness.

## 4. Phase 3 — Horizontal IDOR matrix (Tenant B → Tenant A canaries)

Run every case as **Owner-B requesting Store-A canary objects**, then spot-repeat as **Member-A** (same-tenant, lower-role) where the matrix says so. **One object at a time; stop at first unexpected 200.**

### 4.1 Core object matrix (read)

| ID | Object (Store-A canary) | Request pattern | Expected |
|---|---|---|---|
| IDOR-R01 | Order + line items + delivery/PII | `GET` order detail, items, invoice/receipt endpoints | Neutral 404 (preferred) or 403; **no** fields, no count |
| IDOR-R02 | Product (incl. draft/unpublished) | `GET` product detail by ID/slug; variant/stock endpoints | Same as R01; unpublished must not leak existence |
| IDOR-R03 | Customer record | `GET` customer detail, order history, addresses | Same as R01 |
| IDOR-R04 | Review (pending + published) | `GET` review/moderation endpoints | Same; pending reviews especially sensitive |
| IDOR-R05 | Store settings / storefront / template | `GET` settings, sections, theme, domain config | Same |
| IDOR-R06 | Gateway settings / credentials | `GET` payment-settings endpoints | Same; additionally values must be masked even for *owner* reads except last-4/names |
| IDOR-R07 | Files / media library entries | `GET` file metadata + direct object fetch | Metadata denied; direct public-image URL behavior documented separately (see §6) |
| IDOR-R08 | Invites / members list | `GET` workspace members, pending invites | Same |
| IDOR-R09 | Analytics / visitors / revenue | `GET` stats endpoints, CSV exports | Same; exports must not generate cross-tenant files |
| IDOR-R10 | Notifications / emails log | `GET` notification center, email-history | Same; no recipient addresses cross-tenant |
| IDOR-R11 | Plan / billing / invoices | `GET` subscription, invoices, fee statements | Same |

### 4.2 Core object matrix (write) — higher impact, extra care

Perform writes **only against your own canary objects**, attempting from the *wrong* tenant session. Prefer non-destructive writes (rename canary, toggle a harmless flag); never delete/fulfill/refund anything except your own canaries, and restore afterwards.

| ID | Action (as Owner-B on Store-A canary) | Expected |
|---|---|---|
| IDOR-W01 | Update product (name/price/stock), publish/unpublish | Denied (404-neutral/403); verify via Owner-A session that nothing changed |
| IDOR-W02 | Change order state (fulfill/cancel/refund/note) | Denied; state unchanged |
| IDOR-W03 | Edit/delete customer, merge customers | Denied |
| IDOR-W04 | Approve/reject/delete review | Denied |
| IDOR-W05 | Change store settings, template, domain, policies text | Denied |
| IDOR-W06 | Update/rotate gateway credentials, payout/bank details | Denied; this is the money-path — any 200 here is **Critical**, stop and report |
| IDOR-W07 | Delete/replace files; attach file to foreign product | Denied |
| IDOR-W08 | Invite/remove members, change roles | Denied |
| IDOR-W09 | Change plan, fee acknowledgement, billing reference | Denied |

### 4.3 Technique variants (apply across §4.1–§4.2 — this is where bypasses hide)

| ID | Variant | What to try (with YOUR canary IDs only) | Why it matters |
|---|---|---|---|
| IDOR-T01 | Nested-ID swap | Swap only inner ID (`…/orders/<own>/items/<A-canary>`) and only outer ID | Outer/inner checks often implemented by different code |
| IDOR-T02 | Identifier-type swap | If both slug and UUID/numeric exist, try each; try `id` vs `public_id` vs `handle` params | Alternate lookup paths skip the guard |
| IDOR-T03 | Bulk/array injection | `ids=[own, A-canary]`; CSV `?id=own,A-canary`; batch update/delete | Loops that check only the first/parent object |
| IDOR-T04 | Filter/sort injection | `?org_id=<A>`, `?store_id=<A>`, `?store=<A-handle>`, sort/filter on tenant fields; polluted duplicates (`?org_id=B&org_id=A`) | Tenant taken from request instead of session |
| IDOR-T05 | Method swap | Same path with GET/POST/PUT/PATCH/DELETE/OPTIONS/HEAD | Guards wired to one verb only |
| IDOR-T06 | Suffix/format swap | Append `.json`, trailing slash, `;`, URL-encoding, case flips (`/API/…`) | Router/normalization mismatches |
| IDOR-T07 | Stale-session replay | Replay Owner-A's own captured call *after* logout / after role demotion | Revocation gaps (pairs with S-15) |
| IDOR-T08 | Shopper-context call | Replay merchant API calls with Shopper-S cookies (both stores' shoppers) | Shopper↔merchant boundary |
| IDOR-T09 | Search/export/table endpoints | Store-A canary strings in global search, autocomplete, datatable, CSV/PDF export, "recent items" | Read paths outside CRUD detail views |
| IDOR-T10 | Webhook/callback with foreign refs | Gateway-sandbox callback referencing Store-A order while authed as B (sandbox only) | Order-binding check (pairs with S-05) |

## 5. Phase 4 — Vertical escalation matrix

| ID | Test | Steps | Expected |
|---|---|---|---|
| VERT-01 | Member-A → owner actions | As Member-A: role change, invite/remove, gateway/payout edit, plan change, store delete/transfer | Denied per least-privilege matrix; owner notified of denied sensitive attempts (if alerting exists) |
| VERT-02 | Member-A → member-admin | List all members/invites, resend/revoke invites, view billing | Denied unless role explicitly grants |
| VERT-03 | Shopper-S → merchant | All §4.1 reads + price/cost/margin fields, other shoppers' orders by ID swap | Denied; shopper sees only own orders |
| VERT-04 | Shopper-S → shopper (other) | Order/review/account IDs of the *other* test shopper | Denied |
| VERT-05 | Any merchant → `/admin/*` | Platform-admin routes + admin-flagged API actions | Denied; admin must be a separate session/role, never a client-sent flag |
| VERT-06 | Self-promotion | Attempt `role=owner`, `is_admin=true`, `org_id=<other>` in profile-update, invite-accept, OAuth-link, and signup flows | Ignored; role/org server-assigned only |
| VERT-07 | Invite-accept binding | Accept invite link as a *different* logged-in user / different email case-variant | Bound to invited email; cross-accept denied |

## 6. Phase 5 — File, domain & framework edges

| ID | Test | Steps | Expected |
|---|---|---|---|
| EDGE-01 | Cross-org file metadata | Owner-B requests Store-A file-library entries, thumbnails, signed-URL mint for A-canary file | Denied; no signed URL minted |
| EDGE-02 | Signed-URL tamper (if gated content exists) | Alter expiry/object path/signature; use B's signed URL for A's object; use after revoke/refund | Rejected; entitlement re-checked per request |
| EDGE-03 | Public-image URL handling | Document (don't exploit): which areas are intentionally public; confirm no *gated* content (digital products, invoices) is reachable logged-out | Public images OK by design; gated = signed + entitled only |
| EDGE-04 | Host/tenant resolution | Authorized staging: unknown `Host`, `X-Forwarded-Host` variants, unclaimed subdomain, lapsed custom domain | Neutral 404/landing; never another tenant's store or data |
| EDGE-05 | Next.js Server Actions / RSC | Replay captured Server Action calls cross-tenant (§4 patterns) and unauthenticated | Same guards as REST; no action callable without session+tenant+role check |
| EDGE-06 | Middleware-not-a-gate | Confirm every route/action re-checks authz internally (code review + runtime spot-check), regardless of middleware/version | Pass = defense-in-depth present; record framework version (pairs with S-11) |
| EDGE-07 | Cache poisoning spot-check | With authorization, verify tenant responses aren't served cross-tenant from cache (vary on session/host; no shared cache key for authed data) | No cross-tenant cache hit |

## 7. Pass/fail, oracles & evidence

### 7.1 Verdict rules

- **FAIL (finding)** if a wrong-tenant/wrong-role call returns the canary's data (any field beyond a neutral error), mutates the canary (verify from the owning session), mints a usable signed URL/webhook confirmation, or exports foreign rows.
- **FAIL (oracle)** if existence is distinguishable: e.g. own-missing-ID → `404 {"error":"not_found"}` but foreign-existing-ID → `403 {"error":"forbidden"}` or different timing/size — file as an enumeration oracle (usually Medium, High for customer/order IDs).
- **PASS** if wrong-tenant/wrong-role/anonymous calls get uniform neutral responses (same code, same body shape, same timing class) and the owning session confirms zero mutation.
- **INCONCLUSIVE** if rate-limited/blocked mid-test — record, back off, re-run once; do not hammer.

### 7.2 Severity rubric (for confirmed IDOR only)

| Impact | Severity |
|---|---|
| Read another tenant's orders/customers/PII, gateway/payout settings, or bulk export | **Critical** |
| Write to another tenant (products/orders/settings/reviews/files/members) | **Critical** (money-path W02/W06) / **High** (others) |
| Shopper reads another shopper's orders/PII | **High** |
| Member→owner privilege escalation | **High** |
| Existence oracle on sensitive IDs without data return | **Medium** |
| Anonymous read of non-public tenant data | Raise one level above the authed equivalent |

### 7.3 Evidence pack (per finding)

1. Finding ID + test-case ID + timestamp + environment (staging/prod) + app version/commit.
2. Full request (redacted session tokens) + full response **headers only** + body *shape* (field names / redacted values) — never store foreign PII, even canary-adjacent.
3. Proof-of-ownership: screenshot/log from the *owning* session showing the canary pre/post state (for writes: unchanged = pass; changed = finding).
4. Tool + version, rate used, authorization reference.

### 7.4 Route inventory template (from Phase 1)

`| Route | Method | Object + ID scheme | Caller roles allowed | Tenant check location (file/fn) | Test IDs | Verdict |`
Fill one row per endpoint; an empty "tenant check location" after code review = the finding writes itself.

## 8. What NOT to do (explicit out-of-scope)

- No brute-forcing/guessing real merchant, order, customer, or invite IDs; no sequential-ID sweeps on production.
- No testing against any tenant you did not create, even "just a GET."
- No load, fuzzing storms, or scanner "attack" modes against shared environments; throttle everything (≤ ~1 req/s for manual replays; slower for writes).
- No social engineering of support to resetAYORGEAnyone's account; no phishing; no testing the owner's staff.
- No publishing tenant IDs, URLs, or response samples outside the report's restricted distribution.

## 9. Remediation verification (regression after fixes)

- [ ] Re-run the **entire** §4–§6 matrix (not just the fixed endpoint — guards are usually copy-pasted, and misses cluster).
- [ ] Re-run §7.1 oracle checks (code/message/timing uniformity).
- [ ] Confirm negative automated tests exist per endpoint (wrong-tenant → neutral; wrong-role → deny) and run in CI.
- [ ] Confirm denied cross-tenant attempts emit security-event logs + alerts (pairs with S-12).
- [ ] Sign off: tester, date, version re-tested, residual risks.

## 10. Report snippet template (paste per finding)

```text
[FINDING] <ID> — <Title> — Severity: <Critical/High/Medium/Low>
Scope: <staging/prod> · App version: <…> · Authz ref: <letter/date>
Affected: <route(s), method(s), object(s)>
Repro (own canaries only):
  1. As <alias>: <request description, IDs redacted to pattern>
  2. Observed: <neutral-vs-leak behavior, codes, body shape>
  3. Ownership proof: <owning-session pre/post state>
Impact (if exploited): <who/what, blast radius>
Fix: <server-side tenant+role check location + neutral-response + tests>
Status: <Open / Fixed / Re-tested on <version>>
```

---

*Use this plan only within written authorization and only against tenants you own. If you can read it and it's not yours — stop, don't save it, tell the owner.*
