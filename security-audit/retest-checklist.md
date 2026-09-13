# Re-testing Checklist — storzy.lk & iconnix.lk
Companion to `storzy-iconnix-security-assessment-2026-09-13.md` (§6). Mark each `Pass / Fail / N-A` with redacted evidence.

> Authorization required: Storzy Terms §04 forbids probing without permission. Run §A freely; run §B–§F only with written authorization, on staging + dedicated test tenants (2 merchants + 1 shopper + gateway sandbox).

## A. Baseline — non-intrusive (15 min)
- [ ] Headers captured: apex, www, `/sign-in`, 1× `*.storzy.lk` storefront, 1× custom domain, iconnix (S-09)
- [ ] TLS per host: grade, chain, protocols/ciphers, HSTS, port-80→HTTPS redirect, expiry monitoring (S-10)
- [ ] Cookies: `Secure; HttpOnly; SameSite`, `__Host-` prefix, merchant/shopper separation, lifetimes (S-15)
- [ ] `/.well-known/security.txt` live on both domains (G-01); `robots.txt`/sitemap diffed for new routes
- [ ] Next/React/Node versions + advisories recorded; lockfile + CI gate confirmed (S-11)

## B. Access control & sessions — authorized
- [ ] Wrong-tenant object access → neutral 404 (orders/products/customers/reviews/files/invites/settings) (S-01)
- [ ] Shopper→merchant API + anonymous→`/dashboard /admin /api /invite /s /template` denied neutrally (S-01/S-17)
- [ ] Role matrix enforced server-side incl. Server Actions + bulk endpoints (S-01/S-07)
- [ ] Invite tokens: entropy ≥128-bit, expiry, single-use, revoke, email-binding (S-07)
- [ ] Rotation at login/MFA/privesc; fixation rejected; idle+absolute timeout; logout revokes; JWT strict (S-15)
- [ ] CORS allowlist (no reflect, no `*`+creds); CSRF on mutations; no state-changing GET (S-16)

## C. Money & business logic — gateway sandbox
- [ ] Totals/fees/discounts server-computed; cart tamper rejected (S-02)
- [ ] Stock atomicity, last-unit race, out-of-stock, negative/huge qty rejected (S-02)
- [ ] Forged/replayed/mismatched gateway callbacks rejected; server-to-server confirm mandatory (S-05)
- [ ] Plan limits + fees enforced transactionally (S-02)

## D. Injection, upload & AI — sandbox tenant
- [ ] Stored/reflected/DOM XSS neutralized in all text sinks + reviews + notes (S-03)
- [ ] Upload gauntlet (SVG/HTML/double-ext/magic-byte/oversize/EICAR-benchmark) neutralized; safe serving headers (S-14/S-06)
- [ ] CSP backstop verified with defensive staging PoV (S-09/S-03)
- [ ] Kiki: server-only Gemini, scoped context, injection contained, AI output sanitized pre-apply (S-08)

## E. Auth, domains & ops
- [ ] Password policy + breached-password screen + lockout/throttle + uniform responses (S-04)
- [ ] Reset lifecycle + OAuth state/PKCE/redirect validated (S-04)
- [ ] MFA for merchants + step-up on payout/gateway/role changes (S-04/S-07)
- [ ] Domain claim/verify/re-verify/deprovision + Host handling + cookie scope + cache keys (S-13)
- [ ] R2: no listing, signed+entitled gated content, orphan GC, SVG policy, CDN hostname (S-06)
- [ ] Audit events + redaction + alerts + runbook + notification drill (S-12)
- [ ] iconnix.lk forms/optimizer/`/api/*`/headers (I-01); secrets/SBOM/CI (G-03); DSR/deletion proof (G-02)

## Sign-off
Validator: ________ · Date: ________ · Scope/version tested: ________ · Tool versions: ________ · Residual risks: ________
