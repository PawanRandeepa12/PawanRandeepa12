# Security Assessment — storzy.lk & iconnix.lk
### Black-box, passive, defensive review · OWASP Top 10 (2021) / WSTG / ASVS coverage

| Field | Detail |
|---|---|
| **Targets** | `storzy.lk` (incl. `www`, `*.storzy.lk` merchant subdomains, custom merchant domains) and `iconnix.lk` (incl. `www`) |
| **Date** | 2026-09-13 |
| **Method** | Passive / non-intrusive only: public pages, `robots.txt`, `sitemap.xml`, legal docs, auth-page content, observed cookies/storage/tech. **No login attempts, no payloads, no scans, no exploit attempts.** |
| **Authorization note** | Storzy's Terms of Service §04 explicitly prohibits attacking, overloading, probing, or accessing another merchant's data. Any *active* testing below (Burp/ZAP/nuclei intrusive checks, auth brute-forcing, injection fuzzing) must only be run with **prior written authorization** from Storzy / ICONNIX LK, ideally against staging + a dedicated test tenant. This report is designed so the owners can self-test safely. |
| **Environment limit** | Direct TLS/HTTP probing from this sandbox was blocked at egress (all hosts, including google.com, returned empty/TLS-broken replies), so security headers, TLS config, and cookies could **not** be captured here. Those findings are written as *verify-with-evidence* items with exact commands. Nothing in this report was intrusively validated. |
| **Stack observed** | Next.js + Node.js storefront/SaaS (per ICONNIX case study: multi-tenant, RBAC), Cloudflare R2 file storage (`pub-…​.r2.dev`), PostHog analytics (first-party proxied), Google OAuth + email/password auth, PayHere / Mintpay / Koko / Payzy gateways, Gemini API ("Kiki" AI assistants) |

---

## 1. Executive Summary (for stakeholders)

**In plain language:** Storzy is a multi-tenant e-commerce SaaS (one platform, many merchants' shops, shoppers, orders, and payments), and ICONNIX LK is the agency/marketing site that builds it. The public-facing posture looks *thoughtfully built*: the privacy/cookie documentation is unusually honest and specific (httpOnly cart cookie, separate merchant/shopper sessions, hashed passwords, encrypted gateway credentials, single-use expiring reset links, rate limits on checkout/reviews, no ad trackers). That is a good sign — most small SaaS products document far less.

**But the risk profile is that of a payments-adjacent multi-tenant platform, which is inherently high-stakes.** One authorization bug doesn't leak one shop — it can leak *every* shop. The assessment therefore prioritizes, in order:

1. **Tenant isolation & access control (IDOR/RBAC)** — the #1 hacker target on any multi-tenant SaaS: can Merchant A read/modify Merchant B's orders, products, customers, gateway credentials, or invites by changing an ID in an API call? Can a shopper reach merchant APIs? Can an invited workspace member escalate to admin?
2. **Money-moving logic** — cart/price/quantity/stock tampering, payment-gateway callback forgery (fake "payment succeeded"), plan-limit and fee bypasses.
3. **Stored cross-user scripting (XSS)** — merchant-controlled content (product descriptions, storefront sections, reviews, SVG uploads, AI-suggested designs) rendered in other users' browsers, which can hijack sessions and deface shops.
4. **Account takeover** — password policy (only "8+ characters" disclosed), no advertised MFA or lockout, reset-token and Google-OAuth implementation details.
5. **File & cloud exposure** — a public Cloudflare R2 bucket with predictable per-organization paths; orphaned and digital-product files must not be world-readable by guessable URL.
6. **Unverified transport/baseline hardening** — security headers, cookie flags, TLS config, CORS/CSRF, and dependency patching (especially Next.js) still need evidence capture; none of it could be confirmed passively.

**Bottom line for decision-makers:** no confirmed breach or critical vulnerability is claimed in this report (that requires authorized testing). What *is* confirmed is that the highest-value attack paths are the standard multi-tenant ones, and the report gives the team a prioritized roadmap, concrete fixes with defensive code/config examples, a re-test checklist, and a tool guide — so either the internal team or a contracted pentester can validate and close each item efficiently. Suggested sequencing: **Week 1** — access-control + payment-callback review (needs code access, highest blast radius); **Week 2** — headers/cookies/TLS/MFA/rate-limit hardening (fast, high-signal); **Month 2** — AI/upload/tenant-domain hardening + dependency/SBOM hygiene + logging/alerting.

**Severity spread (21 findings):** 0 Confirmed-Critical · 5 High (1 becomes Critical if validated) · 11 Medium · 5 Low/Info. All High/Medium items above marked "validate" require owner-authorized testing to confirm.

---

## 2. Asset inventory (what was actually observed — the attack surface)

| # | Asset / surface | Observed evidence | Why an attacker cares |
|---|---|---|---|
| A1 | Marketing + app root `www.storzy.lk` | Landing, pricing, templates, `/tools/image-compressor` (client-side, SVG accepted), `/stores` directory | Entry points, upload-adjacent features, info disclosure |
| A2 | Merchant auth: `/sign-up`, `/sign-in`, `/forgot-password`, Google OAuth | "Use at least 8 characters"; reset links "expire after a short time and can only be used once" | Account takeover, enumeration, token attacks |
| A3 | Disclosed private routes (`robots.txt`) | `Disallow: /s/ /dashboard/ /admin/ /api/ /template/ /invite/` | Forced browsing, admin/API discovery |
| A4 | Merchant storefronts `*.storzy.lk` + custom domains (e.g. `www.islandkiks.lk`) | Live tenant shops, `/products/<slug>`, quantity selector, out-of-stock states, reviews tab | Tenant isolation, IDOR, price/stock logic, XSS |
| A5 | Public R2 bucket `pub-78a7c5c4…​.r2.dev/orgs/<org-uuid>/{logos,products,storefront}/<uuid>.<ext>` | Direct image URLs in every storefront page | Bucket listing? Predictable URLs, orphaned/digital files, SVG XSS |
| A6 | Cookies (per cookie policy) | `ph_<id>_posthog` (1-yr random ID, first-party-proxied), `storzy_cart` (httpOnly, 30d), separate merchant/shopper session cookies | Session hijack, fixation, scope (`Domain=.storzy.lk`?), CSRF |
| A7 | Payments: PayHere, Mintpay, Koko, Payzy; "gateway credentials encrypted, key outside DB" | Connector strip + privacy §11 | Callback forgery, amount tampering, credential read-out via IDOR |
| A8 | Kiki AI (2× Gemini features): dashboard/website support chat + editor design assistant ("proposes, you press Apply") | Privacy §06 + FAQ | Prompt injection, cross-tenant data in prompts, AI-output XSS |
| A9 | Workspace invites `/invite/` + "anyone you invite acts with your authority" (Terms §03) | Invite flow exists; no public role matrix | Invite-token guessing, privesc, no least-privilege |
| A10 | Review system ("held for merchant approval, public once published"), cart in cookie, qty/stock controls | Privacy §03, product pages | Stored XSS, moderation bypass, cart tampering |
| A11 | `iconnix.lk` (Next.js: `/_next/image`), `/api/` disallowed, sitemap: services/work/blog/privacy/terms/cookies, contact via email + WhatsApp `wa.me/94753045366` | Marketing site + contact/quote funnel | Form injection, Image-Optimizer cost/SSRF abuse, pivoting/reputation |
| A12 | Missing on both: `/.well-known/security.txt` (404 confirmed), no public disclosure policy | Direct fetch 404 | Slower vuln intake; low severity, easy win |

> No security headers, TLS parameters, or live `Set-Cookie` attributes could be captured from this environment — Section 4/5 shows exactly how to capture them in 5 minutes.

---

## 3. Findings

### 3.1 Severity definitions

| Severity | Meaning in this report |
|---|---|
| **Critical** | Direct path to full tenant/platform compromise or payment theft *once validated*. None confirmed without authorized testing. |
| **High** | Likely path to cross-tenant data access, account takeover, payment fraud, or stored XSS — top fix priority. |
| **Medium** | Meaningful hardening gap or vulnerability class requiring specific conditions; fix on roadmap. |
| **Low / Info** | Defense-in-depth, hygiene, or compliance items; cheap to close. |

Each finding carries a **Confidence** tag: `Observed` (seen in public content) vs `Validate` (requires authorized code/runtime testing). Treat every `Validate` item as "test this first," not as a confirmed hole.

### 3.2 Summary table

| ID | Title | Sev. | Conf. | Maps to (OWASP Top 10 / WSTG / ASVS) |
|---|---|---|---|---|
| S-01 | Cross-tenant IDOR / BOLA across merchant APIs (orders, products, customers, settings, gateway creds, invites) | **High** (Critical if validated) | Validate | A01 / AuthZ, Sess-Mgmt / V4, V13 |
| S-02 | Cart/price/quantity/stock & plan-limit business-logic tampering | **High** | Validate | A01, A04 / Business Logic / V11 |
| S-03 | Stored XSS via merchant content, reviews, SVG uploads, AI-suggested markup | **High** | Validate | A03 / Input Validation, Client-Side / V5 |
| S-04 | Auth hardening gaps: weak disclosed policy, no MFA/lockout evidenced, reset + OAuth edge cases | **High** | Validate | A07 / AuthN, Identity / V2 |
| S-05 | Payment-gateway callback/webhook forgery + amount validation | **High** | Validate | A04, A08 / Business Logic / V11, V8 |
| S-06 | Public R2 URLs: predictable paths, orphaned + digital-product exposure, SVG content-type | Medium (High if digital goods enumerable) | Observed+Validate | A01, A05 / Config / V12, V14 |
| S-07 | Workspace invites & RBAC granularity / vertical escalation | Medium (High if admin client-guarded) | Validate | A01 / AuthZ / V4 |
| S-08 | Kiki AI: prompt injection, over-shared context, AI-output rendering | Medium | Observed+Validate | A04, A03 / Business Logic, Input Val / V5, V11 |
| S-09 | Security headers unverified (CSP, HSTS, frame-ancestors, X-CTO, Referrer/Permissions-Policy, COOP/COEP/CORP) | Medium | Validate | A05 / Config / V14, V9 |
| S-10 | TLS/SSL config unverified (versions, ciphers, chain, OCSP, HSTS preload) | Medium | Validate | A02 / Crypto / V9 |
| S-11 | Next.js/Node supply chain: framework CVEs, deps, lockfiles, third-party scripts | Medium (High if Next unpatched) | Validate | A06, A08 / Config / V10, V14 |
| S-12 | Logging, audit trail & alerting gaps (auth, invites, roles, payout settings) | Medium | Validate | A09 / — / V7 |
| S-13 | Custom-domain verification & subdomain isolation (`*.storzy.lk`, Host header, cookie scope) | Medium | Validate | A01, A05 / Config, Sess-Mgmt / V14, V3 |
| S-14 | File-upload validation (type/magic bytes, SVG, AV, re-encode, disposition) | Medium | Validate | A03, A08 / Input Val / V12 |
| S-15 | Session management (fixation, rotation, timeout, logout, merchant/shopper separation) | Medium | Validate | A07 / Sess-Mgmt / V3 |
| S-16 | CORS & CSRF on APIs/Server Actions (origins, credentials, state-changing GETs) | Medium | Validate | A01 / Sess-Mgmt, Client-Side / V13, V4 |
| S-17 | Error handling & info disclosure (verbose errors, routes, debug endpoints) | Low | Observed+Validate | A05 / Error Handling / V7 |
| I-01 | iconnix.lk: form injection, Image-Optimizer abuse, `/api/` exposure, baseline headers | Low–Medium | Validate | A03, A05 / Input Val, Config / V5, V14 |
| G-01 | No `security.txt` / public disclosure channel on either domain (404 confirmed) | Info | Observed | A09 (process) / — / V1 |
| G-02 | Data-lifecycle review: 7-yr orders, 90-day deletion, orphan files, avatar IDs, analytics proxy | Low | Observed | A04, A09 / — / V8, V1 |
| G-03 | CI/CD & secrets hygiene: Gemini/R2/gateway keys, rotation, SBOM | Medium (process) | Validate | A06, A08 / — / V1, V10 |

### 3.3 Detailed findings

---

#### S-01 — Cross-tenant IDOR / BOLA across merchant APIs — **High** (Critical if validated)
- **Affected component:** `storzy.lk` — `/dashboard/*`, `/api/*`, `/admin/*`, `/invite/*`, `/s/*`, `/template/*`; every tenant-scoped object (orders, products, customers, reviews, gateway settings, files, invites, plan/billing).
- **Description (conceptual):** The ICONNIX case study confirms multi-tenant architecture with RBAC. In this design the single most damaging bug class is a missing or inconsistent tenant-scope check: an authenticated caller supplies another tenant's object ID/handle and the server acts on it without verifying `object.org_id == session.org_id` *and* the caller's role. High-value variants: reading other shops' orders/customer PII, editing products, reading gateway credentials settings, accepting/replaying invites, and reaching `/admin/*` platform functions. Shopper↔merchant confusion (a shopper session calling merchant APIs) is the same bug across role boundaries.
- **Evidence (what to look for — authorized testing only):** with two test merchants (A, B) + one shopper: (1) capture a merchant API call, replay with B's object ID under A's session; (2) test numeric/UUID and slug variants, nested IDs (`/orders/<id>/items/<item>`), bulk endpoints, search/filter/sort params that accept `org_id`/`store_id`; (3) test role matrix: invited member vs owner vs shopper vs anonymous on every endpoint; (4) check Server Actions/RSC the same way as REST — they are endpoints too; (5) look for client-side-only guards (`if (role==='admin')` in JS without server check).
- **Remediation (specific, actionable):** enforce authorization server-side on *every* data access: resolve tenant + role from the session (never from request params), scope all queries (`WHERE org_id = :session_org`), deny-by-default; add centralized helpers (`requireOrgRole()`, `assertOwns(orderId)`) + middleware on all `/api/*` and Server Actions; add negative tests per endpoint (wrong-tenant → 403/404 indistinguishable; never 500 or partial data); normalize "not found vs forbidden" responses to avoid oracle; log denied cross-tenant attempts as security events.
- **References:** OWASP Top 10 A01:2021; WSTG Authorization Testing (OTG-AUTHZ-001/002/004); ASVS V4.1/V4.2, V13.4; CWE-639 (Authorization Bypass Through User-Controlled Key), CWE-285, CWE-862.

#### S-02 — Cart/price/quantity/stock & plan-limit business-logic tampering — **High**
- **Affected component:** Storefront cart (`storzy_cart` cookie), checkout, `/products/<slug>` qty controls, stock checks, plan limits/fees.
- **Description:** Cart lives client-side (cookie) and product pages expose quantity/stock UI — classic tamper points. If the server trusts client-supplied price, quantity bounds, stock state, discounts, or plan-limit checks, an attacker can underpay, oversell phantom stock, apply other tenants' promos, or bypass "read-only when over limits" and per-order fees. Time-of-check/time-of-use races on low-stock items can oversell.
- **Evidence:** authorized tests: modify cart cookie (qty 0/negative/huge, price fields if present, foreign product IDs, other-store items mixed in); add out-of-stock items; replay checkout; race two checkouts for last unit; exceed plan limits (products/ storage/emails) as free-tier merchant; verify totals/fees recomputed server-side from DB prices.
- **Remediation:** never trust client totals — recompute price/tax/fee/discount server-side from authoritative product rows inside a transaction; validate qty as positive int within min/max and available stock with atomic decrement (`UPDATE … WHERE stock >= qty`); re-check stock at payment-confirm time, not just add-to-cart; sign or server-store carts (or HMAC cart cookies); enforce plan limits + fees in the same transaction; add idempotency keys on order creation.
- **References:** A01/A04; WSTG Business Logic Testing; ASVS V11; CWE-472 (External Control of Assumed-Immutable Data), CWE-367 (TOCTOU), CWE-20.

#### S-03 — Stored XSS via merchant content, reviews, SVG uploads, AI markup — **High**
- **Affected component:** Product descriptions, storefront sections/templates, reviews (merchant-approved → public), uploaded SVGs/images, Kiki design-assistant output, order notes, store names/handles.
- **Description:** Anywhere Merchant A's text reaches Shopper B's browser (or Owner B's dashboard) is a stored-XSS sink: product HTML, section content, review name/body, file names/alt text, AI-proposed markup applied via "Apply". SVG uploads are executable markup. Impact: session theft, shop defacement, payment-page skimming, admin-session compromise.
- **Evidence:** authorized tests only, in a sandbox tenant: submit script-event-handler/`javascript:`/`data:`/SVG-embedded probes into every text + upload field; check output contexts (HTML, attribute, JS, URL, CSS); verify whether `dangerouslySetInnerHTML` is used; confirm `Content-Type`/disposition of served uploads; confirm AI suggestions are sanitized before preview/apply/publish.
- **Remediation:** context-aware output encoding everywhere; sanitize rich HTML server-side with an allowlist (e.g. sanitize-html/DOMPurify-on-server) and re-sanitize on render; never `dangerouslySetInnerHTML` with unsanitized input; serve uploads with `Content-Type: image/*` + `X-Content-Type-Options: nosniff` + `Content-Disposition: attachment` for non-renderables; rasterize or strip scripts from SVGs (prefer converting to PNG/WebP; if SVG must render inline, sanitize + serve from a separate cookie-less asset host); add a strict CSP (see S-09) as backstop; moderate + encode reviews.
- **References:** A03; WSTG Input Validation (OTG-INPVAL-002) & Client-Side; ASVS V5; CWE-79, CWE-116.

#### S-04 — Authentication hardening gaps — **High**
- **Affected component:** `/sign-up`, `/sign-in`, `/forgot-password`, Google OAuth, session issuance.
- **Description:** Public surface discloses only "8+ characters" — no MFA, lockout, breach-password screening, or rate-limit statement for auth (rate limits are only claimed for checkout/reviews). Merchant-account takeover cascades to shop content, orders/PII, and payout/gateway settings. Ancillary risks: user enumeration via timing/messages, reset-token entropy/expiry/reuse, Host-header reset-link poisoning, OAuth `redirect_uri`/`state` validation.
- **Evidence:** verify: password policy (length/complexity/breached-password check via k-Anonymity), lockout + throttling on sign-in/reset (incl. per-IP *and* per-account with uniform responses), reset token ≥128-bit, hashed at rest, single-use, short expiry, invalidated after use/password change, reset emails use absolute canonical URLs (ignore `Host`/`X-Forwarded-Host`); OAuth `state`+PKCE, exact redirect-URI match, account-linking takeover checks; no enumeration delta (messages + timing) across existing/non-existing emails.
- **Remediation:** offer TOTP/WebAuthn MFA (at least for merchants; require for plan/payout/gateway changes = step-up auth); strict rate limit + CAPTCHA-safe backoff on auth endpoints; uniform auth responses; Argon2id/bcrypt password hashing with server-side pepper; breached-password screening; secure reset-token lifecycle; security notifications (new device, password/reset, email change); admin/owner session re-auth for sensitive actions.
- **References:** A07; WSTG Authentication/Identity Testing; ASVS V2; CWE-307, CWE-308, CWE-640, CWE-798, NIST SP 800-63B.

#### S-05 — Payment-gateway callback/webhook forgery + amount validation — **High**
- **Affected component:** PayHere / Mintpay / Koko / Payzy integrations; order state machine (`pending → paid → fulfilled`).
- **Description:** The money moment: if "payment succeeded" can be asserted client-side (redirect params, hidden fields, replayed callbacks) or gateway signatures aren't verified against gateway-side truth, orders can be marked paid without payment. Related: currency/amount mismatch, duplicate-callback double-fulfilment, merchant gateway-credential read/update without owner-only checks.
- **Evidence:** authorized: attempt order-state transitions via UI/API without gateway proof; tamper amount/currency/order-id in every step; replay old callbacks; send unsigned/forged callbacks; verify server reconciles via gateway verify-API (server-to-server) before marking paid; confirm idempotent webhook handling; confirm gateway-credential settings are owner-only, masked on read, encrypted (as documented) with no plaintext echo.
- **Remediation:** treat gateway redirect as *untrusted hint only*; confirm payment via server-to-server verify call + webhook signature check (HMAC with gateway secret, constant-time compare, timestamp tolerance, replay store); bind `order_id↔amount↔currency` server-side; idempotency keys + unique constraint on gateway refs; least-privilege credential access + rotation runbook; alert on verify-mismatch spikes.
- **References:** A04/A08; WSTG Business Logic; ASVS V11/V8; CWE-345 (Insufficient Verification of Data Authenticity), CWE-807; gateway docs (PayHere/Koko/Mintpay/Payzy signature verification).

#### S-06 — Public R2 object exposure — **Medium** (High if digital goods enumerable)
- **Affected component:** `pub-….r2.dev/orgs/<org>/<area>/<uuid>.<ext>` — logos, products, storefront, size charts, share images, **digital products**.
- **Description (observed):** Every storefront embeds direct public R2 URLs with a stable, guessable structure (org UUID + area + file UUID). UUIDs are unguessable individually, but: (a) any leaked/shared URL is world-readable forever; (b) "orphaned" uploads persist until account close (privacy §09) — deleted-product images stay public; (c) digital products on the same public scheme = paid content without access control; (d) SVG served as `image/svg+xml` executes script in shoppers' browsers (links S-03); (e) the `r2.dev` host discloses the storage backend.
- **Evidence:** check: bucket listing disabled; no sequential/short IDs; orphan lifecycle; whether digital-product URLs work logged-out/after-refund; `Content-Type`/`nosniff`/disposition headers on objects; whether a custom CDN domain with signed URLs is available for gated content.
- **Remediation:** keep *public* images public but move *gated* content (digital products, invoices, size charts if sensitive) to signed/presigned URLs with short expiry + entitlement check; delete or tombstone orphaned objects promptly (reference-count GC, not just at account close); block SVG execution (serve `Content-Security-Policy: sandbox` or rasterize; never `image/svg+xml` inline on the app origin); migrate public delivery to a first-party CDN hostname; document retention in merchant DPA.
- **References:** A01/A05; WSTG Configuration & Deployment; ASVS V12/V14; CWE-538, CWE-552, CWE-434.

#### S-07 — Workspace invites & RBAC granularity — **Medium** (High if `/admin` is client-guarded)
- **Affected component:** `/invite/*`, workspace member management, `/admin/*`, role checks in APIs/Server Actions.
- **Description:** Terms §03 says invitees "act with your authority" — if roles are binary (in/out) or unenforced server-side, any invited member can change gateway/payout settings, invite others, or read PII. Invite links themselves are bearer credentials: short/guessable, non-expiring, or multi-use tokens let attackers join workspaces.
- **Evidence:** map the role matrix (owner/admin/member/shopper/anonymous × every sensitive action); test invite-token entropy/expiry/single-use/max-uses/domain-restriction; test invite replay after accept/revoke; test direct API calls to owner-only actions as member; test `/admin/*` + `/api/*` as non-admin.
- **Remediation:** least-privilege roles (Owner / Manager / Staff-read-only or finer), enforced server-side per action; invite tokens ≥128-bit, single-use, ≤72h expiry, bound to email, revocable, auditable; re-auth + notify owner on role/invite changes; never expose admin APIs to non-admin sessions (route + data-layer checks).
- **References:** A01; WSTG Authorization Testing; ASVS V4; CWE-269, CWE-862, CWE-613.

#### S-08 — Kiki AI: prompt injection, context overshare, output rendering — **Medium**
- **Affected component:** Support chat (dashboard + public site) and editor design assistant → Google Gemini API.
- **Description (observed + validate):** Documented design is already careful (least context, no order/PII to Gemini, no training, transcript in browser only, human "Apply" gate). Residual risks: (1) indirect prompt injection — attacker text already in the store (product/review/section content, help-article-like pages) steering the assistant; (2) support-chat answers leaking one merchant's setup state in a confused/multi-store session; (3) AI-proposed design diffs containing malicious markup/URLs applied to a public shop; (4) API-key exposure client-side; (5) abuse/cost (prompt flooding).
- **Evidence:** verify all Gemini calls are server-side with key in vault; verify session→store binding for the "setup summary"; test injection probes in store content that the assistant might quote; test that AI output is sanitized/allow-listed before preview/apply/publish; check rate limits + abuse monitoring + output logging policy (PII-free).
- **Remediation:** keep the human gate; treat all AI output as untrusted input (sanitize + allowlist diff ops: only known section/style fields); hard tenant scoping on context assembly; system-prompt hardening + delimiters; server-side calls only; per-account rate limits + cost alerts; never render AI markdown/HTML raw; publish AI-data terms (already largely done — keep accurate).
- **References:** A04/A03; OWASP LLM Top 10 (LLM01 Prompt Injection, LLM06 Sensitive Info Disclosure, LLM08 Excessive Agency); ASVS V5/V11; CWE-1427, CWE-918 (if assistant fetches URLs).

#### S-09 — Security headers unverified — **Medium**
- **Affected component:** All `storzy.lk` origins (app, storefronts, custom domains) + `iconnix.lk`.
- **Description:** Headers are the cheapest XSS/clickjacking/downgrade defense and could not be captured here (sandbox egress). Missing/weak CSP, HSTS, framing, or MIME-sniffing controls amplify S-03/S-15/S-16.
- **Evidence:** capture (owner/operator, 5 min): `curl -sSI https://www.storzy.lk/`, a tenant storefront, `/sign-in`, and `https://www.iconnix.lk/`; run Mozilla Observatory + securityheaders.com; confirm per-tenant and custom-domain parity.
- **Remediation:** ship baseline (tune CSP `connect-src` for PostHog-proxy/Gemini/gateway hosts): `Content-Security-Policy`, `Strict-Transport-Security: max-age=63072000; includeSubDomains; preload` (after readiness), `X-Content-Type-Options: nosniff`, `Referrer-Policy: strict-origin-when-cross-origin`, `Permissions-Policy` (camera/mic/geo off), framing via CSP `frame-ancestors 'none'` (app) / allowlisted (embeddable storefronts only if needed), `Cross-Origin-Opener/Embedder/Resource-Policy` tightened where compatible; `X-Frame-Options: DENY` legacy fallback. See §5 for copy-paste config.
- **References:** A05; WSTG Configuration (OTG-CONFIG-007/008); ASVS V14.4/V9; CWE-693, CWE-1021, CWE-79 (defense-in-depth).

#### S-10 — TLS/SSL configuration unverified — **Medium**
- **Affected component:** All HTTPS endpoints incl. `*.storzy.lk` and merchant custom domains.
- **Description:** Privacy §11 states "traffic is served over HTTPS" — good claim, needs evidence: protocol/cipher hygiene, certificate chain + SNI for tenant/custom domains, HTTP→HTTPS redirect, HSTS/preload, OCSP.
- **Evidence:** SSL Labs scan per hostname (apex, www, sample tenant subdomain, sample custom domain); `testssl.sh` or `openssl s_client` protocol/cipher checks; verify port-80 redirect (301 → canonical HTTPS, no content served over HTTP); cert expiry monitoring.
- **Remediation:** TLS 1.2+ (prefer 1.3), strong cipher suites only, valid full chain, auto-renewal + expiry alerting, HSTS preload when stable, OCSP stapling, disable compression/renegotiation issues; terminate consistently at CDN/LB *and* origin (no HTTP hop in between).
- **References:** A02; WSTG SSL/TLS Testing; ASVS V9; CWE-295, CWE-326, CWE-327.

#### S-11 — Next.js / Node supply chain & framework patching — **Medium** (High if Next unpatched)
- **Affected component:** App + marketing site dependencies, build/deploy pipeline, third-party scripts/CDN.
- **Description:** Next.js has had critical framework-level issues (notably CVE-2025-29927 middleware authorization bypass; past RSC/SSRF/cache-poisoning classes). An outdated framework or vulnerable transitive dep re-opens S-01/S-03/S-16 regardless of app code. No SBOM/lockfile evidence is publicly visible.
- **Evidence:** record Next/React/Node versions; `npm audit` / OSV-Scanner / Snyk; verify lockfile committed + CI fails on critical advisories; confirm middleware does not gate auth alone (re-verify inside routes/actions); inventory third-party scripts (avatars, PostHog proxy, gateway SDKs) with SRI/`async` and CSP allowlisting.
- **Remediation:** upgrade Next.js to a patched release immediately if affected; pin + lock + Dependabot/Renovate; fail builds on critical CVEs; verify auth inside every route/Server Action (defense in depth beyond middleware); minimize third-party JS; generate and retain an SBOM per release.
- **References:** A06/A08; OWASP Dependency-Check / SCVS; ASVS V10/V14; CWE-1104, CVE-2025-29927 (verify current Next security releases).

#### S-12 — Logging, audit trail & alerting gaps — **Medium**
- **Affected component:** Auth, invites/roles, product/order/price changes, gateway settings, AI actions, webhook verifies.
- **Description:** No public signal of audit logging beyond rate-limit counters. Without an immutable trail + alerts, S-01/S-02/S-04/S-05 exploitation is silent. Over-logging PII/secrets is the opposite failure (privacy §05 already sets a good "minimal email metadata" precedent — extend it).
- **Evidence:** confirm events logged: sign-in (success/fail), reset request/use, MFA changes, invite create/accept/revoke, role changes, gateway credential changes, order state transitions + verify results, denied cross-tenant attempts; confirm no passwords/tokens/keys/PII bodies in logs; confirm retention + access controls + alert rules.
- **Remediation:** structured security-event log (who/what/when/source/result), tamper-evident retention, PII/secret redaction, log-injection neutralization (strip CR/LF), dashboards + alerts (verify-mismatch spikes, invite/role churn, auth bursts, R2 egress anomalies), incident runbook + breach-notification path (privacy §11 already promises notification — operationalize it).
- **References:** A09; WSTG Error Handling/Logging; ASVS V7; CWE-778, CWE-117, CWE-532.

#### S-13 — Custom-domain verification & subdomain isolation — **Medium**
- **Affected component:** Custom merchant domains, `*.storzy.lk` routing, tenant resolution, cookie scope.
- **Description:** "Verified custom domains" are promised — verification must be airtight (DNS TXT/ownership + re-verification, no takeover of AJORGEak/expired domains), tenant resolution must never fall through to another store on unknown `Host`, and cookies/sessions must not leak across `*.storzy.lk` or onto custom domains. Wildcard TLS + shared JS/CSS across tenants also widen cache-poisoning/XSS blast radius.
- **Evidence:** test unknown/spoofed `Host` + `X-Forwarded-Host` handling; attempt claiming another merchant's domain/subdomain; check cookie `Domain` scoping (host-only where possible), CORS `Access-Control-Allow-Origin` reflection, cache keys including `Host`; confirm expired-domain deprovisioning.
- **Remediation:** strict allowlist tenant resolution (unknown host → neutral 404, no data); DNS-ownership verification + periodic re-check + fast deprovision; host-only cookies; `Vary: Host` + correct cache keys; forbid `X-Forwarded-Host` overrides (trust only the platform LB header).
- **References:** A01/A05; WSTG Config & Session Mgmt; ASVS V13/V14; CWE-918, CWE-441, CWE-20.

#### S-14 — File-upload validation — **Medium**
- **Affected component:** Product photos, logos, share images, size charts, digital products, (adjacent: image-compressor accepts SVG client-side).
- **Description:** Uploads are the classic RCE/XSS bridge: type confusion (SVG/HTML-as-image), polyglots, oversized/bomb files, executable extensions, AV-evasive malware in digital products resold to shoppers.
- **Evidence:** authorized: upload SVG-with-script, HTML-renamed, double-extension, magic-byte-mismatched, oversized, and EICAR-benchmark files to each upload path; verify server-side allowlist (extension + magic bytes + re-encode), size caps, AV verdict, randomized object names, and serving headers.
- **Remediation:** server-side allowlist + magic-byte sniffing + image re-encode (strip metadata/scripts), hard size caps, AV scan for digital goods, random unguessable keys, least-privilege R2 credentials (per-prefix write), safe serving headers (S-06), per-plan storage quotas enforced server-side.
- **References:** A03/A08; WSTG Input Validation; ASVS V12; CWE-434, CWE-436.

#### S-15 — Session management — **Medium**
- **Affected component:** Merchant + shopper sessions, sign-out, "remember me" if any, JWT if used.
- **Description:** Cookie policy claims separate, mutually-unreadable merchant/shopper sessions — verify it holds at the `Domain`/`Path`/`__Host-` level and server-side (no shared token namespace, no role field trusted from client). Classic gaps: fixation (no rotation at login), no idle/absolute timeout, logout not revoking server-side, concurrent-session abuse, JWT `alg:none`/confusion/expiry.
- **Evidence:** test rotation at login/MFA/privilege change; fixation (pre-login cookie reuse); idle + absolute expiry; logout invalidation (replay old cookie); cross-role replay (shopper cookie on merchant API and vice versa); cookie flags (`Secure; HttpOnly; SameSite=Lax/Strict; Path=/; __Host-` prefix); if JWT: `alg`, `exp`, `aud/iss`, key handling.
- **Remediation:** rotate on auth-state change; server-side session store with revoke-on-logout/password-change; idle (~30 min merchant) + absolute (~12 h) timeouts; `__Host-`-prefixed, `Secure/HttpOnly/SameSite` cookies; re-auth for sensitive actions; JWT only if needed, with strict validation.
- **References:** A07; WSTG Session Management; ASVS V3; CWE-384, CWE-613, CWE-614, CWE-287.

#### S-16 — CORS & CSRF on APIs/Server Actions — **Medium**
- **Affected component:** `/api/*`, Server Actions/mutations, PostHog analytics proxy, webhook URLs.
- **Description:** Credentialed cross-origin trust (`Access-Control-Allow-Origin` reflecting `Origin` + `Allow-Credentials: true`, or `*` with credentials) lets malicious sites read merchant/shopper data; missing CSRF protection on cookie-authed mutations lets them *change* it (order state, products, settings). State-changing GETs and unprotected analytics proxies compound it.
- **Evidence:** send `Origin: https://evil.example` (authorized, own test account) to API + proxy endpoints; check reflected origin + credentials; test cross-site POST of a mutation with cookies but no CSRF token; confirm no state change over GET; confirm webhooks require signatures (not cookies).
- **Remediation:** explicit origin allowlist (never reflect; never `*` + credentials); `SameSite=Lax` (or Strict for dashboard) + CSRF tokens/double-submit or `Origin`/`Sec-Fetch-*` validation on mutations; GET = safe/idempotent only; lock down analytics-proxy paths/methods.
- **References:** A01; WSTG Session/CSRF/CORS; ASVS V4/V13; CWE-352, CWE-942, CWE-639.

#### S-17 — Error handling & information disclosure — **Low**
- **Affected component:** App/API error paths, 404s, `robots.txt`-listed routes, Next error overlays.
- **Description (observed + validate):** `robots.txt` advertises sensitive prefixes (necessary for crawlers, but also a discovery list — acceptable *if* everything behind them is auth-guarded). Residual risk is verbose/production error leakage (stack traces, SQL/ORM errors, paths, versions) aiding attackers.
- **Evidence:** trigger 404/400/500/auth-fail paths as anonymous + authed; confirm generic messages + correlation IDs, no traces/paths/SQL; confirm production (not dev) error pages on all origins incl. tenant subdomains and custom domains.
- **Remediation:** global error boundary (generic user message + server-side-logged correlation ID); disable debug/stack in production; keep `robots.txt` but ensure every listed area returns 401/403/404-neutral anonymously; remove sample/debug endpoints; add `404` handling that doesn't echo unsanitized input (reflected XSS check).
- **References:** A05; WSTG Error Handling; ASVS V7; CWE-209, CWE-497.

#### I-01 — iconnix.lk marketing-site hardening — **Low–Medium**
- **Affected component:** `www.iconnix.lk` pages/forms, `/_next/*` optimizer, `/api/*` (robots-disallowed).
- **Description:** Lower data sensitivity than Storzy, but compromise = defacement, SEO spam, contact-form abuse, credential-phishing under a trusted brand, and pivot credibility. Specific probes: contact/quote-form injection (header injection → spam relay; stored XSS if enquiries render in an admin UI); `/_next/image?url=` remote-URL fetching (cost/SSRF-adjacent if open); `/api/*` exposure; baseline headers/TLS (same S-09/S-10 checklist); WhatsApp/email harvesting is inherent — ensure no extra PII (staff, internal URLs) in pages/JS bundles.
- **Evidence:** map forms + `/api/*`; test header/XSS probes in authorized staging; check `next.config` image `remotePatterns`; capture headers/TLS; grep bundles for keys/endpoints.
- **Remediation:** validate + encode all form input; no email-header passthrough; rate-limit + CAPTCHA-safe throttling on forms; lock image remote patterns to owned hosts; auth-guard or remove unnecessary `/api/*`; apply §5 header/TLS baseline; strip secrets from client bundles.
- **References:** A03/A05; WSTG Input Validation & Config; ASVS V5/V14; CWE-79/93/918.

#### G-01 — No published vulnerability-disclosure channel — **Info**
- **Affected component:** Both domains (`/.well-known/security.txt` → 404 confirmed on both).
- **Description (observed):** No `security.txt` and no documented reporting address/process. Researchers and customers who find issues have no safe, monitored path — reports go to generic inboxes or public.
- **Evidence:** direct fetch of `/.well-known/security.txt` on both hosts returns the app 404 page (this report's only directly-verified negative).
- **Remediation:** publish `security.txt` (contact, expires, policy URL, PGP optional), add a security/disclosure page, define SLA + safe-harbor wording, route to a monitored alias. (Example in §5.)
- **References:** RFC 9116; OWASP DSOMM; ASVS V1.1 (secure SDLC alignment).

#### G-02 — Data-lifecycle & privacy-engineering review — **Low**
- **Affected component:** Order retention (7 y), deletion (90 d), orphan files, avatar IDs, analytics proxy.
- **Description (observed):** Policies are transparent; residual engineering checks: 7-year order/tax retention needs access minimization; 90-day deletion needs proof (incl. backups/R2/analytics); orphan uploads linger to account close (§09); placeholder avatars derive from "account identifier" (must be a non-enumerable hash, not a raw ID); PostHog-via-own-domain proxy must not become an open forwarder and should honor opt-out/DNT signals.
- **Evidence:** verify DSR runbook (access/export/delete ≤30 d), backup purge, R2 GC, avatar-ID irreversibility, analytics-proxy path allowlist, cookie-banner/consent parity for the non-necessary analytics cookie.
- **Remediation:** automate deletion/GC with evidence logs; hash + salt avatar seeds; proxy allowlist + method/path lockdown; document sub-processors for merchant DPAs (already offered — keep current); PDPA/GDPR RoPA alignment.
- **References:** A04/A09 (process); ASVS V8; Sri Lanka PDPA No. 9 of 2022; GDPR Art. 17/28/30.

#### G-03 — CI/CD & secrets hygiene — **Medium (process)**
- **Affected component:** Build/deploy pipeline, Gemini API keys, R2 credentials, gateway secrets, session/cookie secrets, SBOM.
- **Description:** The documented "encryption key held outside the DB" is good — extend the pattern: no secrets in repos/client bundles/CI logs, per-environment vaulting, rotation, least-privilege deploy tokens, signed builds, SBOM per release.
- **Evidence:** grep history/bundles for keys; confirm vault/KMS usage, rotation dates, CI OIDC (no long-lived tokens), branch protection + required reviews, secret-scanning (gitleaks/trufflehog) in CI.
- **Remediation:** vault + rotation runbook + break-glass; secret scanning + push protection; SLSA-considered builds; SBOM (CycloneDX) stored per release; dependency-update automation (links S-11).
- **References:** A06/A08; OWASP SCVS/DSOMM; ASVS V1/V10; CWE-798, CWE-312.

---

## 4. Prioritized Remediation Roadmap

### 🔴 Phase 0 — Before anything active (Day 0)
- [ ] Get **written authorization** defining scope (prod vs staging, tenants, rate limits, forbidden actions), emergency contacts, and rollback. (Required by Storzy Terms §04.)
- [ ] Stand up **staging + 2 test merchants + shopper accounts + test gateway sandbox**; never pentest prod tenants.
- [ ] Capture baseline evidence (15 min): headers/TLS/cookies per §6.1.

### 🟥 Phase 1 — Quick wins, highest blast radius (Week 1–2) — kills the hacker top-3
| Priority | Action | Closes/supports |
|---|---|---|
| P0 | Server-side tenant+role enforcement pass on all `/api/*` + Server Actions + `/admin/*` (S-01), with negative tests | S-01, S-07, S-16 |
| P0 | Gateway verify-server-side + signed-webhook + amount/currency binding + idempotency (S-05) | S-05, S-02 |
| P0 | Server-side totals/stock-atomicity + cart integrity (S-02) | S-02 |
| P1 | Ship security-header baseline + cookie flags + error-page hardening (S-09, S-15, S-17) | S-09, S-15, S-17, S-03(depth) |
| P1 | Auth throttling/lockout + reset-token audit + OAuth `state`/redirect audit; ship MFA plan (S-04) | S-04 |
| P1 | Next.js/framework + dep upgrade triage; middleware-not-a-gate review (S-11) | S-11, S-01 |
| P1 | Publish `security.txt` + disclosure SLA (G-01) | G-01 |

### 🟨 Phase 2 — Structural fixes (Week 3–6)
| Priority | Action | Closes/supports |
|---|---|---|
| P1 | Stored-XSS program: sanitize/encode pass, SVG policy, upload re-encode + serving headers (S-03, S-14, S-06) | S-03, S-14, S-06 |
| P1 | Signed URLs + entitlement for digital/gated files; orphan GC (S-06) | S-06, G-02 |
| P1 | RBAC least-privilege + invite-token lifecycle + re-auth for sensitive actions (S-07) | S-07, S-01 |
| P2 | CORS/CSRF tightening + analytics-proxy lockdown + no-state-changing-GET (S-16) | S-16 |
| P2 | Session lifecycle (rotation/timeout/revoke, JWT strictness) (S-15) | S-15, S-04 |
| P2 | AI hardening: server-only Gemini, context scoping, output sanitization, abuse limits (S-08) | S-08 |
| P2 | Domain verification + Host-handling + cookie scoping + cache keys (S-13) | S-13 |

### 🟩 Phase 3 — Long-term / continuous (Month 2+)
- [ ] TLS hygiene automation + HSTS preload readiness + expiry monitoring (S-10).
- [ ] Security-event logging/alerting + runbooks + breach-notification drill (S-12).
- [ ] CI/CD vaulting/rotation, secret scanning, SBOM per release, Dependabot gating (G-03, S-11).
- [ ] DSR/deletion-proof automation, avatar-seed hashing, retention minimization (G-02).
- [ ] iconnix.lk form/optimizer/API pass + shared header/TLS baseline (I-01).
- [ ] Annual (or per-major-release) authorized pentest + quarterly dependency/audit review; bug-bounty or researcher-friendly disclosure once basics are green.

**Suggested ownership:** P0 items need backend engineers with code access; P1 header/TLS/cookie items are DevOps-friendly; S-08/S-13 need platform/architecture input; G-items suit security/process owners.

---

## 5. Secure Configuration & Code Examples (defensive only)

> Drop-in, conservative baselines. Adjust hosts to your own; test in staging first.

### 5.1 Next.js security headers (`next.config.js`)
```js
// Defensive baseline — sets headers for app + storefront origins.
const securityHeaders = [
  { key: 'Strict-Transport-Security', value: 'max-age=63072000; includeSubDomains; preload' },
  { key: 'X-Content-Type-Options', value: 'nosniff' },
  { key: 'Referrer-Policy', value: 'strict-origin-when-cross-origin' },
  { key: 'Permissions-Policy', value: 'camera=(), microphone=(), geolocation=(), payment=()' },
  { key: 'X-Frame-Options', value: 'DENY' }, // legacy fallback; CSP frame-ancestors is authoritative
  { key: 'Cross-Origin-Opener-Policy', value: 'same-origin' },
  { key: 'Cross-Origin-Resource-Policy', value: 'same-origin' },
  {
    key: 'Content-Security-Policy',
    value: [
      "default-src 'self'",
      "base-uri 'self'",
      "frame-ancestors 'none'", // relax ONLY for deliberately embeddable storefront paths
      "form-action 'self'",
      "object-src 'none'",
      "img-src 'self' https: data:",           // tighten 'https:' to your CDN/R2 + avatar hosts
      "font-src 'self' data:",
      "style-src 'self' 'unsafe-inline'",       // remove unsafe-inline if your CSS allows
      "script-src 'self'",                      // add nonces/hashes if inline scripts are needed
      "connect-src 'self' https://www.storzy.lk https://storzy.lk", // + PostHog-proxy path, gateway + Gemini hosts ONLY via server
      'upgrade-insecure-requests',
    ].join('; '),
  },
];

module.exports = {
  async headers() {
    return [{ source: '/:path*', headers: securityHeaders }];
  },
  images: {
    // I-01: never allow arbitrary remote image hosts (cost/SSRF-adjacent).
    remotePatterns: [{ protocol: 'https', hostname: 'cdn.storzy.lk' }],
  },
};
```

### 5.2 Session cookies (server-issued, defensive)
```js
// Node/Next route handler — cookie flags that kill most session-theft/XSS-follow-on impact.
res.setHeader('Set-Cookie', [
  `__Host-storzy_sid=${sessionId}; Path=/; Secure; HttpOnly; SameSite=Lax; Max-Age=43200`, // 12h absolute cap
  // Keep merchant vs shopper namespaces SEPARATE (names + server-side stores), host-only (no Domain=).
]);
// + rotate session ID on login/MFA/privilege change; revoke server-side on logout/password change.
```

### 5.3 Tenant + role enforcement helper (conceptual guard — every data path)
```js
// Illustration of the S-01/S-07 pattern: tenant + role resolved from SESSION, never request params.
async function requireOrgRole(session, orderId, allowed = ['owner', 'manager']) {
  const order = await db.orders.findById(orderId);      // fetch first…
  if (!order || order.orgId !== session.orgId) throw httpError(404); // …neutral 404 (no cross-tenant oracle)
  if (!allowed.includes(session.role)) throw httpError(403);
  return order;
}
// Apply inside EVERY route handler AND Server Action (middleware alone is not sufficient).
```

### 5.4 Webhook signature verification (gateway callbacks — S-05 pattern)
```js
// Verify-then-act: raw body HMAC, constant-time compare, timestamp tolerance, replay store.
import crypto from 'node:crypto';
function verifyWebhook({ rawBody, signature, secret, maxSkewSec = 300, store }) {
  const [ts, mac] = String(signature).split('.', 2);
  if (Math.abs(Date.now() / 1000 - Number(ts)) > maxSkewSec) return false;
  const expect = crypto.createHmac('sha256', secret).update(`${ts}.${rawBody}`).digest('hex');
  if (!crypto.timingSafeEqual(Buffer.from(expect), Buffer.from(mac))) return false;
  return store.claimOnce(`${ts}:${mac}`); // reject replays; then confirm amount/currency via gateway API
}
// State rule: only transition pending→paid AFTER server-to-server confirmation; idempotency key per gateway ref.
```

### 5.5 Atomic stock decrement + server-side totals (S-02 pattern)
```sql
-- Single statement: only succeeds if stock covers the request (kills oversell races).
UPDATE products SET stock = stock - :qty
 WHERE id = :id AND org_id = :org AND stock >= :qty;
-- Then: if rowcount = 0 → reject (insufficient stock); recompute totals from DB prices in the same transaction.
```

### 5.6 Output encoding / sanitization (S-03 pattern)
```js
// Rich text: sanitize server-side with an allowlist; never dangerouslySetInnerHTML on raw input.
import sanitizeHtml from 'sanitize-html';
const clean = sanitizeHtml(dirty, {
  allowedTags: ['p','br','strong','em','ul','ol','li','a','h2','h3'],
  allowedAttributes: { a: ['href','rel','target'] },
  allowedSchemes: ['https'],
  transformTags: { a: (tag, attrs) => ({ tag, attrs: { ...attrs, rel: 'noopener nofollow', target: '_blank' } }) },
});
// Plain text in JSX is auto-escaped — keep it that way; URLs/attrs need explicit validation.
```

### 5.7 `security.txt` (G-01 — serve at `/.well-known/security.txt`)
```text
Contact: mailto:security@storzy.lk
Expires: 2027-03-01T00:00:00.000Z
Preferred-Languages: en, si
Policy: https://www.storzy.lk/legal/security
# Mirror for iconnix.lk with its own Contact/Policy lines.
```

### 5.8 Baseline capture commands (evidence for S-09/S-10/S-15/S-16)
```bash
# Run from any machine with normal egress (sandbox here was blocked), authorized scope only.
curl -sSI https://www.storzy.lk/ | sed -n '1,40p'
curl -sSI https://www.storzy.lk/sign-in | grep -iE '^(HTTP|strict|content-security|x-|referrer|permissions-|cross-|set-cookie)'
curl -sS -D- -o /dev/null https://lilloop.storzy.lk/ | grep -iE '^(HTTP|strict|content-security|x-|set-cookie)'
curl -sSI https://www.iconnix.lk/ | sed -n '1,40p'
curl -sS -o /dev/null -w 'redirect:%{redirect_url} code:%{http_code}\n' http://storzy.lk/
# CORS probe (own test account, authorized):
curl -sSI -H 'Origin: https://evil.example' https://www.storzy.lk/api/health 2>/dev/null | grep -i 'access-control'
# TLS (or use SSL Labs UI): testssl.sh --fast https://www.storzy.lk
```

---

## 6. Re-testing Checklist

> Copy-paste ready for the validator. Every intrusive step requires the Phase-0 authorization. Mark each `Pass/Fail/N-A` with evidence (request/response excerpts redacted of secrets).

### 6.1 Baseline (non-intrusive, anyone can run)
- [ ] Headers captured for apex, www, `/sign-in`, 1 tenant subdomain, 1 custom domain, iconnix (S-09).
- [ ] TLS: SSL Labs grade, chain, protocols/ciphers, HSTS, port-80 redirect, expiry monitoring (S-10).
- [ ] Cookies: flags + `__Host-` + merchant/shopper separation + lifetimes (S-15, cookie-policy parity).
- [ ] `security.txt` live on both domains (G-01). `robots.txt`/sitemap reviewed for new routes.
- [ ] Framework + runtime versions recorded (Next/React/Node), advisories triaged (S-11).

### 6.2 Access control & sessions (authorized test tenants)
- [ ] Wrong-tenant object access → neutral 404 for orders/products/customers/reviews/files/invites/settings (S-01).
- [ ] Shopper session on merchant APIs → denied; anonymous on `/dashboard /admin /api /invite /s /template` → neutral deny (S-01/S-17).
- [ ] Role matrix enforced server-side (owner/manager/staff/shopper) incl. Server Actions + bulk endpoints (S-01/S-07).
- [ ] Invite tokens: entropy/expiry/single-use/revoke/email-binding verified (S-07).
- [ ] Session rotation/timeout/logout-revoke/fixation + JWT strictness (if any) (S-15).
- [ ] CORS allowlist (no reflect/`*`+creds) + CSRF on mutations + no state-changing GET (S-16).

### 6.3 Money & business logic (gateway sandbox)
- [ ] Totals/fees/discounts recomputed server-side; cart tamper (price/qty/foreign items) rejected (S-02).
- [ ] Stock atomicity + last-unit race + out-of-stock + negative/huge qty rejected (S-02).
- [ ] Gateway callback forgery/replay/amount-currency mismatch rejected; server-to-server confirm required (S-05).
- [ ] Plan limits + fees enforced transactionally; no free-tier bypass (S-02).

### 6.4 Injection, upload & AI (sandbox tenant)
- [ ] Stored/reflected/DOM XSS probes neutralized in every text sink + review flow + order notes (S-03).
- [ ] Uploads: SVG/HTML/double-ext/magic-byte/oversize/EICAR-benchmark rejected or neutralized; safe serving headers (S-14/S-06).
- [ ] CSP blocks inline/event-handler exfil in staging PoV (defensive validation) (S-09/S-03).
- [ ] Kiki: server-only Gemini, scoped context, injection probes contained, AI output sanitized pre-apply (S-08).

### 6.5 Auth, domain & ops
- [ ] Password policy + breached-password screen + lockout/throttle + uniform responses (S-04).
- [ ] Reset lifecycle (entropy/hash/expiry/single-use/invalidate-on-change, canonical URLs) + OAuth state/PKCE/redirect (S-04).
- [ ] MFA available (merchants) + step-up for payout/gateway/role changes (S-04/S-07).
- [ ] Custom-domain claim/verify/re-verify/deprovision + Host-header + cookie-scope + cache-key review (S-13).
- [ ] R2: no listing, gated content signed + entitled, orphan GC, SVG policy, CDN hostname (S-06).
- [ ] Audit events + redaction + alerts + runbook + notification drill (S-12).
- [ ] iconnix.lk forms/optimizer/`/api/*`/headers pass (I-01); secrets/SBOM/CI gates (G-03); DSR/deletion evidence (G-02).

**Sign-off:** validator name, date, scope, commit/version tested, tool versions, residual risks.

---

## 7. Appendix: Tools & how to interpret output

> **Scope rule for every active tool:** authorized targets/tenants only, staging preferred, throttle enabled, stop on anomalies. Passive tools (header/TLS/version checks) are safe to run against public URLs.

| Tool | Use it for | How to run (authorized) | How to read output |
|---|---|---|---|
| **Burp Suite (Pro/Community)** | Intercept/replay for S-01/S-02/S-05/S-16; session/auth matrix | Scope to in-scope hosts; Repeater for IDOR/cart/callback replays; Comparer for oracle diffs | Wrong-tenant `200` + foreign data = fail; `403/404`-neutral + no data = pass; any `500`/trace = info-leak fail |
| **OWASP ZAP** | Baseline + authenticated spider/passive scan; CSRF/header alerts | `zap-baseline.py -t https://staging…`; then authenticated ajax-spider on test tenant | Triage High/Medium; manually confirm each (ZAP false-positives on Next/RSC) — never auto-attack prod |
| **nuclei** | CVE/misconfig templates (headers, exposures, CVEs) — **passive-safe templates first** | `nuclei -u https://host -t cves,misconfiguration,exposures -rate-limit 20` (authorized) | Any `critical/high` template hit → verify manually; version-detect hits feed S-11 |
| **testssl.sh / SSL Labs** | S-10 evidence | `testssl.sh --fast https://host`; SSL Labs per hostname | Require TLS1.2+, strong ciphers, full chain, HSTS; anything `E/F`, weak cipher, or chain break = fail |
| **securityheaders.com / Mozilla Observatory** | S-09 evidence | Scan each origin incl. tenant + custom domain | Missing CSP/HSTS/nosniff = findings; grade deltas across origins = parity bug |
| **npm audit / OSV-Scanner / Snyk / Dependabot** | S-11, G-03 | `npm audit --omit=dev`; `osv-scanner -r .`; Snyk monitor; Dependabot alerts on | Fix critical/high with exploit maturity first; confirm lockfile + CI gate; regenerate SBOM |
| **Semgrep / CodeQL** | S-01/S-03/S-05 patterns in code (white-box) | Rules: `javascript`, `owasp-top-ten`, `auth`, `xss`, `ssrf` packs on PRs | Every `dangerouslySetInnerHTML`, unscoped query, unsigned webhook path = review queue |
| **gitleaks / trufflehog** | G-03 secret hygiene | Scan history + staged + bundles | Any key/token in repo/bundle/CI log = rotate + purge + post-mortem |
| **Playwright / curl scripts** | Repeatable re-tests (§6) for auth/session/cart/roles | Script the §6 matrix against staging; assert status + neutrality | Green suite = regression net for every release; red = release blocker for P0/P1 |
| **PostHog/proxy + browser DevTools** | G-02/S-16 cookie + proxy review | Network tab: cookie flags, proxy paths, `Origin`/CORS headers | `Secure/HttpOnly/SameSite` missing, `Domain=.storzy.lk` over-broad, proxy forwarding = findings |
| **sqlmap — NOT for blind use** | Only if a specific injectable param is suspected *and* explicitly authorized | Prefer code review + parameterized-query audit first | Any confirmed injection = Critical; default to prepared statements everywhere instead |
| **feroxbuster/gobuster — restricted** | Only authorized staging, tiny wordlist, throttled | Prefer sitemap + `robots.txt` + JS-bundle route harvest (passive) | New admin/debug/backup endpoints = investigate; aggressive fuzzing can trip §04/abuse controls |

**Evidence hygiene for every tool:** save command + version + timestamp + redacted output; file per finding ID; re-run after fixes for §6 sign-off.

---

## 8. Limits, assumptions & next steps

- **Not a pentest.** No authenticated testing, fuzzing, or exploitation was performed or attempted. Findings marked `Validate` are prioritized hypotheses with exact test procedures — confirmation needs owner-authorized access.
- **Assumptions:** standard Next.js/Node multi-tenant SaaS; R2 via public `r2.dev` + app-mediated gating claims unverified; gateway integrations per public connector names; Gemini via server-side calls (unverified).
- **Suggested next step:** share this report with the Storzy/ICONNIX engineering owner, agree Phase-0 authorization, run §6.1 (15 min, no auth needed), then schedule the Phase-1 code+runtime review. A focused 3–5 day authorized assessment should be able to confirm-or-close every High item.
- **Responsible handling:** this report contains no exploit code and no shopper/merchant data. Keep distribution to stakeholders + validators; do not publish per-tenant details.

*Prepared 2026-09-13 · Scope: passive review of public storzy.lk & iconnix.lk surfaces · Standards: OWASP Top 10:2021, WSTG, ASVS.*
