#!/usr/bin/env bash
# IDOR regression harness — SAFE TEMPLATE. Defaults to the LOCAL mock only.
# Reuse vs your own staging: set BASE_URL + session/canary vars below, with written authorization,
# only against tenants you own. Never point at production tenants that aren't yours.
#
#   BASE_URL=https://staging.example.com SESSION_A='...' SESSION_B='...' \
#     CANARY_A_ORDER=ord-A1 CANARY_B_ORDER=ord-B1 CANARY_A_MARK=CANARY-A-cx7q2 ./idor-regression.sh
set -u
BASE_URL="${BASE_URL:-http://127.0.0.1:8901}"
AUTH_HEADER="${AUTH_HEADER:-x-test-session}"   # mock uses header; for staging use Cookie via COOKIE_* vars
SESSION_A="${SESSION_A:-ownerA}"               # Owner-A session value (mock: alias; staging: cookie string)
SESSION_B="${SESSION_B:-ownerB}"               # Owner-B session value
CANARY_A_ORDER="${CANARY_A_ORDER:-ord-A1}"
CANARY_B_ORDER="${CANARY_B_ORDER:-ord-B1}"
CANARY_A_MARK="${CANARY_A_MARK:-CANARY-A-cx7q2}"

PASS=0; FAIL=0
req() { # $1=method $2=path $3=session-or-empty $4=data-or-empty -> "CODE BODY"
  local args=(-s -m 10 -o /tmp/h_body -w '%{http_code}' -X "$1" "$BASE_URL$2")
  [ -n "${3:-}" ] && args+=(-H "$AUTH_HEADER: $3")
  [ -n "${4:-}" ] && args+=(-H 'content-type: application/json' -d "$4")
  local code; code=$(curl "${args[@]}")
  echo "$code $(cat /tmp/h_body)"
}
check() { # $1=test-id $2=description $3=expected $4=actual
  if [ "$3" = "$4" ]; then PASS=$((PASS+1)); echo "PASS  $1  $2";
  else FAIL=$((FAIL+1)); echo "FAIL  $1  $2  (expected [$3] got [$4])"; fi
}
leakcheck() { # $1=test-id $2=description $3=body  -> fails if A's canary mark present
  if echo "$3" | grep -q "$CANARY_A_MARK"; then FAIL=$((FAIL+1)); echo "FAIL  $1  $2  (Store-A canary LEAKED cross-tenant)";
  else PASS=$((PASS+1)); echo "PASS  $1  $2"; fi
}

echo "== IDOR regression vs $BASE_URL =="

# BASE-01: own access works (harness sanity)
R=$(req GET "/api/orders/$CANARY_A_ORDER" "$SESSION_A"); CODE=${R%% *}; BODY=${R#* }
check "BASE-01" "Owner-A reads own order (200 + canary)" "200+canary" "$CODE+$([ "${BODY#*"$CANARY_A_MARK"}" != "$BODY" ] && echo canary || echo nocanary)"

# BASE-02: anonymous denied
R=$(req GET "/api/orders/$CANARY_A_ORDER" ""); CODE=${R%% *}; BODY=${R#* }
check "BASE-02" "Anonymous denied without data" "401" "$CODE"; leakcheck "BASE-02b" "Anonymous response has no canary" "$BODY"

# IDOR-R01: B reads A's order
R=$(req GET "/api/orders/$CANARY_A_ORDER" "$SESSION_B"); CODE=${R%% *}; BODY=${R#* }
[ "$CODE" = "404" ] || [ "$CODE" = "403" ] && V=denied || V="$CODE"; check "IDOR-R01" "Owner-B read of Store-A order denied" "denied" "$V"
leakcheck "IDOR-R01b" "Cross-tenant read leaks no canary" "$BODY"

# Oracle check: foreign-existing vs missing ID must be indistinguishable
MISS=$(req GET "/api/orders/ord-NOPE" "$SESSION_B")
FOREIGN=$(req GET "/api/orders/$CANARY_A_ORDER" "$SESSION_B")
check "ORACLE-01" "Foreign-existing vs missing ID identical (code+body)" "$MISS" "$FOREIGN"

# IDOR-W02: B writes A's order note (non-destructive canary field) -> denied + unchanged
R=$(req PUT "/api/orders/$CANARY_A_ORDER/note" "$SESSION_B" '{"note":"PWNED-BY-B"}'); CODE=${R%% *}
[ "$CODE" = "404" ] || [ "$CODE" = "403" ] && V=denied || V="$CODE"; check "IDOR-W02" "Owner-B write to Store-A order denied" "denied" "$V"
R=$(req GET "/api/orders/$CANARY_A_ORDER" "$SESSION_A"); BODY=${R#* }
echo "$BODY" | grep -q "PWNED-BY-B" && V=mutated || V=unchanged; check "IDOR-W02b" "Store-A canary unchanged (owner re-read)" "unchanged" "$V"

# IDOR-T04: org_id filter injection honored?
R=$(req GET "/api/orders?org_id=orgA" "$SESSION_B"); BODY=${R#* }
leakcheck "IDOR-T04" "org_id=orgA as Owner-B returns no Store-A rows" "$BODY"

# VERT-05: merchant reaches /admin
R=$(req GET "/api/admin/users" "$SESSION_B"); CODE=${R%% *}; BODY=${R#* }
[ "$CODE" = "404" ] || [ "$CODE" = "403" ] || [ "$CODE" = "401" ] && V=denied || V="$CODE"; check "VERT-05" "Owner-B denied /admin" "denied" "$V"

echo "== result: $PASS passed, $FAIL failed =="
[ "$FAIL" -eq 0 ]
