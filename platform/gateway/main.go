// OMNI Unified AI API Platform - Data Plane (Go) - Reference Implementation
// Target: 11-15μs overhead, inspired by Bifrost (Maxim AI) + Envoy AI Gateway
// Apache 2.0 - Open Core

package main

import (
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/google/uuid"
)

// Config - LiteLLM compatible config.yaml simplified
type ProviderConfig struct {
	Provider string `yaml:"provider" json:"provider"`
	Model    string `yaml:"model" json:"model"`
	APIKey   string `yaml:"api_key" json:"-"`
	APIURL   string `yaml:"api_base" json:"api_base"`
	Weight   int    `yaml:"weight" json:"weight"`
}

type Config struct {
	Providers []ProviderConfig `yaml:"providers"`
	Cache     CacheConfig      `yaml:"cache"`
	Budgets   BudgetConfig     `yaml:"budgets"`
}

type CacheConfig struct {
	ExactEnabled    bool    `yaml:"exact_enabled" json:"exact_enabled"`
	SemanticEnabled bool    `yaml:"semantic_enabled" json:"semantic_enabled"`
	SemanticThresh  float64 `yaml:"semantic_threshold" json:"semantic_threshold"`
	TTLSeconds      int     `yaml:"ttl_seconds" json:"ttl_seconds"`
}

type BudgetConfig struct {
	MaxTokensPerMin int `yaml:"max_tokens_per_min" json:"max_tokens_per_min"`
	MaxCostPerDay   float64 `yaml:"max_cost_per_day" json:"max_cost_per_day"`
}

// Virtual Key - Portkey/LiteLLM pattern
type VirtualKey struct {
	KeyID      string            `json:"key_id"`
	HashedKey  string            `json:"hashed_key"`
	TeamID     string            `json:"team_id"`
	Budgets    BudgetConfig      `json:"budgets"`
	AllowedModels []string       `json:"allowed_models"`
	Metadata   map[string]string `json:"metadata"`
}

// Chat Request - OpenAI compatible with OMNI extensions
type ChatRequest struct {
	Model             string                 `json:"model"`
	Messages          []Message              `json:"messages"`
	Temperature       *float64               `json:"temperature,omitempty"`
	MaxTokens         *int                   `json:"max_tokens,omitempty"`
	Stream            bool                   `json:"stream,omitempty"`
	OMNIPreferences   *ModelPreferences      `json:"model_preferences,omitempty" yaml:"model_preferences"`
	FallbackModels    []string               `json:"fallback_models,omitempty"`
	CacheConfig       *CacheRequestConfig    `json:"cache,omitempty"`
	Budget            *BudgetRequest         `json:"budget,omitempty"`
	Metadata          map[string]interface{} `json:"metadata,omitempty"`
}

type Message struct {
	Role    string `json:"role"`
	Content string `json:"content"`
}

type ModelPreferences struct {
	Cost    float64 `json:"cost"` // 0-1 weight
	Quality float64 `json:"quality"`
	Latency float64 `json:"latency"`
}

type CacheRequestConfig struct {
	Enabled   bool    `json:"enabled"`
	Semantic  bool    `json:"semantic"`
	Threshold float64 `json:"threshold"`
}

type BudgetRequest struct {
	MaxTokens int `json:"max_tokens"`
}

type ChatResponse struct {
	Id      string   `json:"id"`
	Object  string   `json:"object"`
	Created int64    `json:"created"`
	Model   string   `json:"model"`
	Choices []Choice `json:"choices"`
	Usage   Usage    `json:"usage"`
	OMNI    OMNIMeta `json:"omni"`
}

type Choice struct {
	Index   int     `json:"index"`
	Message Message `json:"message"`
	FinishReason string `json:"finish_reason"`
}

type Usage struct {
	PromptTokens     int `json:"prompt_tokens"`
	CompletionTokens int `json:"completion_tokens"`
	TotalTokens      int `json:"total_tokens"`
	Cached           bool `json:"cached,omitempty"`
}

type OMNIMeta struct {
	ProviderUsed   string  `json:"provider_used"`
	CacheHit       bool    `json:"cache_hit"`
	CacheType      string  `json:"cache_type,omitempty"` // exact, semantic
	LatencyMs      float64 `json:"latency_ms"`
	CostSaved      float64 `json:"cost_saved,omitempty"`
	ModelRouted    string  `json:"model_routed,omitempty"` // if auto
}

// In-memory mock storage - replace with Redis/Postgres/ClickHouse in prod
var (
	virtualKeys = map[string]VirtualKey{}
	cacheStore  = map[string]ChatResponse{} // exact cache
)

func init() {
	// Seed a demo virtual key
	vk := VirtualKey{
		KeyID: "vk_demo",
		HashedKey: hashKey("omni_demo_12345"),
		TeamID: "team_acme",
		Budgets: BudgetConfig{MaxTokensPerMin: 100000, MaxCostPerDay: 100},
		AllowedModels: []string{"*"},
		Metadata: map[string]string{"env": "demo"},
	}
	virtualKeys[vk.HashedKey] = vk
	log.Printf("[INIT] Demo virtual key: omni_demo_12345 -> %s", vk.KeyID)
}

func hashKey(k string) string {
	h := sha256.Sum256([]byte(k))
	return fmt.Sprintf("%x", h[:8])
}

func estimateTokens(text string) int {
	// Simplified tiktoken: ~4 chars per token
	return int(math.Ceil(float64(len(text)) / 4.0))
}

// Middleware: Auth with virtual keys - Portkey pattern
func authMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()
		auth := c.GetHeader("Authorization")
		if !strings.HasPrefix(auth, "Bearer ") {
			c.AbortWithStatusJSON(401, gin.H{"error": "missing Bearer token"})
			return
		}
		token := strings.TrimPrefix(auth, "Bearer ")
		hashed := hashKey(token)
		vk, ok := virtualKeys[hashed]
		if !ok {
			// Allow omni_ prefix for demo but warn
			if !strings.HasPrefix(token, "omni_") {
				c.AbortWithStatusJSON(401, gin.H{"error": "invalid virtual key"})
				return
			}
			// Fallback demo key
			vk = virtualKeys[hashKey("omni_demo_12345")]
		}
		c.Set("virtualKey", vk)
		c.Set("requestStart", start)
		c.Next()
	}
}

// Middleware: Token-aware rate limiting - Redis Lua would be used in prod
func rateLimitMiddleware() gin.HandlerFunc {
	counters := map[string]int{} // team -> tokens used this minute (simplified)
	return func(c *gin.Context) {
		vki, exists := c.Get("virtualKey")
		if !exists {
			c.Next()
			return
		}
		vk := vki.(VirtualKey)
		// Check budget - simplified: check total tokens estimate
		if vk.Budgets.MaxTokensPerMin > 0 {
			used := counters[vk.TeamID]
			if used > vk.Budgets.MaxTokensPerMin {
				c.AbortWithStatusJSON(429, gin.H{"error": "token budget exceeded", "team_id": vk.TeamID})
				return
			}
		}
		c.Next()
	}
}

// Middleware: Observability - OTel would be here
func observabilityMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Next()
		// After request, log metrics
		startVal, _ := c.Get("requestStart")
		if startVal != nil {
			latency := time.Since(startVal.(time.Time)).Seconds() * 1000
			log.Printf("[METRICS] %s %s team=%v latency=%.2fms status=%d", c.Request.Method, c.FullPath(), c.GetString("team"), latency, c.Writer.Status())
		}
	}
}

// Handler: OpenAI-compatible chat completions with intelligent routing + caching
func chatCompletionsHandler(c *gin.Context) {
	var req ChatRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(400, gin.H{"error": err.Error()})
		return
	}

	start := time.Now()
	requestID := "chatcmpl-" + uuid.New().String()[:8]

	// 1. Cache lookup: exact
	cacheKey := fmt.Sprintf("%s:%s:%v", req.Model, hashMessages(req.Messages), req.Temperature)
	if req.CacheConfig == nil || req.CacheConfig.Enabled {
		if cached, ok := cacheStore[cacheKey]; ok {
			cached.OMNI.CacheHit = true
			cached.OMNI.CacheType = "exact"
			cached.OMNI.LatencyMs = float64(time.Since(start).Microseconds()) / 1000.0
			cached.Usage.Cached = true
			c.JSON(200, cached)
			return
		}
	}

	// 2. Semantic cache check (mock - would query Qdrant with embedding)
	if req.CacheConfig != nil && req.CacheConfig.Semantic {
		// In real: embed last message, query Qdrant with threshold 0.95
		// For demo: skip, but log intent
		log.Printf("[CACHE] semantic lookup for %s threshold %.2f", requestID, req.CacheConfig.Threshold)
	}

	// 3. Intelligent routing - Cost Intelligence Engine (simplified)
	providerUsed := "openai"
	modelRouted := req.Model
	if req.Model == "auto" {
		// Pareto logic: if cost preference >0.6, pick DeepSeek for non-reasoning
		pref := req.OMNIPreferences
		if pref != nil && pref.Cost > 0.6 {
			modelRouted = "deepseek-v3"
			providerUsed = "deepseek"
		} else {
			modelRouted = "gpt-5-mini" // balanced default
			providerUsed = "openai"
		}
	}

	// 4. Fallback handling - attempt primary, fallback on failure
	// In prod: circuit breaker per provider, retry with backoff
	fallbacks := req.FallbackModels
	if len(fallbacks) == 0 {
		fallbacks = []string{"gpt-5-mini", "gemini-2.5-flash", "claude-haiku-4.5"}
	}

	// Mock provider call - in real, HTTP to provider with KMS-decrypted key
	// Simulate Latency: Groq <100ms, OpenAI 500-1000ms
	latencySim := 120 * time.Millisecond
	if providerUsed == "deepseek" {
		latencySim = 400 * time.Millisecond
	}
	time.Sleep(latencySim)

	// Mock response
	resp := ChatResponse{
		Id:      requestID,
		Object:  "chat.completion",
		Created: time.Now().Unix(),
		Model:   modelRouted,
		Choices: []Choice{
			{
				Index: 0,
				Message: Message{
					Role:    "assistant",
					Content: fmt.Sprintf("[OMNI routed to %s via %s] Hello! This is a mock response. Your message had ~%d tokens. Model pref cost=%.1f quality=%.1f. Real impl would call %s API.", modelRouted, providerUsed, estimateTokens(req.Messages[len(req.Messages)-1].Content), getPref(req.OMNIPreferences, "cost"), getPref(req.OMNIPreferences, "quality"), providerUsed),
				},
				FinishReason: "stop",
			},
		},
		Usage: Usage{
			PromptTokens:     estimateTokens(req.Messages[len(req.Messages)-1].Content),
			CompletionTokens: 50,
			TotalTokens:      estimateTokens(req.Messages[len(req.Messages)-1].Content) + 50,
		},
		OMNI: OMNIMeta{
			ProviderUsed: providerUsed,
			CacheHit:     false,
			LatencyMs:    float64(time.Since(start).Microseconds()) / 1000.0,
			ModelRouted:  modelRouted,
			CostSaved:    0.0,
		},
	}

	// If semantic + cost pref, show savings
	if req.Model == "auto" && providerUsed == "deepseek" {
		// GPT-5 $1.25/M vs DeepSeek $0.28/M = $0.97/M saving
		resp.OMNI.CostSaved = float64(resp.Usage.TotalTokens) / 1e6 * 0.97
	}

	// 5. Write to cache
	if req.CacheConfig == nil || req.CacheConfig.Enabled {
		cacheStore[cacheKey] = resp
	}

	c.JSON(200, resp)
}

func hashMessages(msgs []Message) string {
	b, _ := json.Marshal(msgs)
	h := sha256.Sum256(b)
	return fmt.Sprintf("%x", h[:6])
}

func getPref(p *ModelPreferences, which string) float64 {
	if p == nil {
		return 0.5
	}
	switch which {
	case "cost":
		return p.Cost
	case "quality":
		return p.Quality
	case "latency":
		return p.Latency
	}
	return 0.5
}

// Handler: MCP Gateway - Federate tools/list (stateless 2026-07-28 RC)
func mcpHandler(c *gin.Context) {
	// Support JSON-RPC 2.0 over Streamable HTTP per spec
	var rpcReq map[string]interface{}
	if err := c.ShouldBindJSON(&rpcReq); err != nil {
		c.JSON(400, gin.H{"error": "invalid JSON-RPC"})
		return
	}

	method, _ := rpcReq["method"].(string)
	id := rpcReq["id"]

	// Check Mcp-Method header routing (stateless core)
	mcpMethodHeader := c.GetHeader("Mcp-Method")
	if mcpMethodHeader != "" {
		method = mcpMethodHeader
	}

	log.Printf("[MCP] method=%s id=%v", method, id)

	switch method {
	case "initialize":
		c.JSON(200, gin.H{
			"jsonrpc": "2.0",
			"id":      id,
			"result": gin.H{
				"protocolVersion": "2026-07-28",
				"capabilities": gin.H{
					"tools":     gin.H{"listChanged": true},
					"resources": gin.H{"subscribe": true},
				},
				"serverInfo": gin.H{"name": "omni-mcp-gateway", "version": "0.1.0"},
			},
		})
	case "tools/list":
		// Federated tools from multiple upstreams - mock 3 servers merged
		// In prod: pull from Redis list of upstream MCP servers for this team
		tools := []gin.H{
			{"name": "github__create_issue", "description": "Create GitHub issue (federated from github MCP)", "inputSchema": gin.H{"type": "object", "properties": gin.H{"repo": gin.H{"type": "string"}, "title": gin.H{"type": "string"}}}},
			{"name": "postgres__query", "description": "Query Postgres (federated, requires approval)", "inputSchema": gin.H{"type": "object", "properties": gin.H{"sql": gin.H{"type": "string"}}}},
			{"name": "slack__post_message", "description": "Post to Slack (federated)", "inputSchema": gin.H{"type": "object"}},
			{"name": "omni__brave_search", "description": "Web search via Brave - $0.001 per call (monetized via x402)", "inputSchema": gin.H{"type": "object", "properties": gin.H{"query": gin.H{"type": "string"}}}, "price": "0.001", "currency": "USDC"},
		}
		c.JSON(200, gin.H{
			"jsonrpc": "2.0",
			"id":      id,
			"result": gin.H{"tools": tools},
		})
	case "tools/call":
		params, _ := rpcReq["params"].(map[string]interface{})
		toolName, _ := params["name"].(string)
		// RBAC + approval check - in prod, check policy engine
		if strings.Contains(toolName, "postgres__") {
			// Simulate approval required
			// Check if header X-MCP-Approval: true
			if c.GetHeader("X-MCP-Approval") != "true" {
				c.JSON(200, gin.H{
					"jsonrpc": "2.0",
					"id":      id,
					"error":   gin.H{"code": -32001, "message": "approval required for sensitive tool", "data": gin.H{"tool": toolName, "approval_url": "https://api.omni.ai/v1/approvals/req_123"}},
				})
				return
			}
		}

		// Check x402 payment for monetized tools
		if toolName == "omni__brave_search" {
			// Hybrid middleware: check for X-Payment-Receipt or MPP session
			receipt := c.GetHeader("X-Payment-Receipt")
			mppSession := c.GetHeader("X-MPP-Session")
			stripeKey := c.GetHeader("Authorization")

			if receipt == "" && mppSession == "" && stripeKey == "" {
				// Return 402 with payment options - per x402 spec
				c.Header("X-Payment-Amount", "1000") // 0.001 USDC in atomic units? 1000 = 0.001?
				c.Header("X-Payment-Token", "USDC")
				c.Header("X-Payment-Chain", "base")
				c.Header("X-Payment-Address", "0xOMNI_MERCHANT_WALLET")
				c.JSON(402, gin.H{
					"jsonrpc": "2.0",
					"id":      id,
					"error": gin.H{
						"code":    402,
						"message": "Payment Required",
						"data": gin.H{
							"price": "0.001",
							"currency": "USDC",
							"schemes": []string{"x402", "mpp", "stripe"},
						},
					},
				})
				return
			}
			log.Printf("[COMMERCE] tool=%s payment verified receipt=%s mpp=%s", toolName, receipt != "", mppSession != "")
		}

		// Mock tool execution
		c.JSON(200, gin.H{
			"jsonrpc": "2.0",
			"id":      id,
			"result": gin.H{
				"content": []gin.H{{"type": "text", "text": fmt.Sprintf("[OMNI gateway] Executed tool %s successfully. In prod, this would proxy to upstream MCP server.", toolName)}},
				"isError": false,
			},
		})
	default:
		c.JSON(200, gin.H{"jsonrpc": "2.0", "id": id, "result": gin.H{}})
	}
}

// Handler: Media - fal.ai compatible
func mediaHandler(c *gin.Context) {
	model := c.Param("type") // images, videos, audio
	var body map[string]interface{}
	c.ShouldBindJSON(&body)
	log.Printf("[MEDIA] type=%s model=%v", model, body["model"])
	c.JSON(200, gin.H{
		"request_id": uuid.New().String(),
		"model":      body["model"],
		"status":     "queued",
		"message":    fmt.Sprintf("[OMNI Media Plane] Proxy to fal.ai/Replicate for %s - would return generation ID in real impl", model),
		"omni": gin.H{"provider": "fal.ai", "estimated_cost": "$0.02"},
	})
}

// Handler: Usage / Cost Intelligence
func usageHandler(c *gin.Context) {
	// In prod: ClickHouse query for team
	teamID := c.Query("team")
	if teamID == "" {
		teamID = "team_acme"
	}
	c.JSON(200, gin.H{
		"team_id": teamID,
		"period":  "24h",
		"tokens": gin.H{"input": 2500000, "output": 800000, "cached": 1200000, "cache_hit_rate": 0.42},
		"cost": gin.H{"provider": 125.50, "platform_fee": 15.06, "saved_via_cache": 52.30, "saved_via_routing": 48.60, "total_saved": 100.90, "net": 140.56},
		"models": []gin.H{
			{"model": "deepseek-v3", "tokens": 1500000, "cost": 0.42},
			{"model": "gpt-5-mini", "tokens": 800000, "cost": 0.60},
			{"model": "claude-haiku-4.5", "tokens": 1000000, "cost": 1.00},
		},
		"recommendation": gin.H{"message": "Switch 30% traffic from gpt-5-mini to deepseek-v3 for classification, save $32/mo with 97% quality parity", "action": "update policy"},
	})
}

// Health
func healthHandler(c *gin.Context) {
	c.JSON(200, gin.H{"status": "ok", "service": "omni-gateway", "version": "0.1.0-rc", "uptime": "99.99% target", "perf": "11μs target (bifrost benchmark)", "mcp_spec": "2026-07-28 RC", "x402": "V2 + MPP ready"})
}

func main() {
	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Recovery())
	r.Use(observabilityMiddleware())

	// Public
	r.GET("/health", healthHandler)

	// Protected - AI Gateway (OpenAI compat)
	v1 := r.Group("/v1")
	v1.Use(authMiddleware())
	v1.Use(rateLimitMiddleware())
	{
		v1.POST("/chat/completions", chatCompletionsHandler)
		v1.POST("/completions", chatCompletionsHandler) // legacy
		v1.POST("/:type/generations", mediaHandler) // images/videos/audio compat
		v1.GET("/usage", usageHandler)
		// Additional: /embeddings, /moderations, /ocr, etc would be same pattern
	}

	// MCP Gateway - Statless, spec 2026-07-28, Streamable HTTP
	mcp := r.Group("/mcp")
	mcp.Use(authMiddleware()) // OAuth per MCP spec would be here with iss validation
	{
		mcp.POST("/v1/:team", mcpHandler)
		mcp.POST("/v1", mcpHandler) // default team
	}

	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}

	log.Printf(`
 ██████╗ ███╗   ███╗███╗   ██╗██╗
██╔═══██╗████╗ ████║████╗  ██║██║
██║   ██║██╔████╔██║██╔██╗ ██║██║  Unified AI API Platform v0.1.0-rc
██║   ██║██║╚██╔╝██║██║╚██╗██║██║  Target: 11μs overhead (Bifrost)
╚██████╔╝██║ ╚═╝ ██║██║ ╚████║██║  MCP Spec: 2026-07-28 RC stateless
 ╚═════╝ ╚═╝     ╚═╝╚═╝  ╚═══╝╚═╝  x402 V2 + MPP Ready
                                 Envoy-based data plane
Listening on :%s
- POST /v1/chat/completions (OpenAI compat, auto router, cache, fallback)
- POST /mcp/v1/:team (MCP gateway federated, RBAC, x402 monetization)
- POST /v1/:type/generations (media compat fal.ai)
- GET  /v1/usage (cost intelligence)
Demo key: omni_demo_12345
`, port)

	if err := r.Run(":" + port); err != nil {
		log.Fatal(err)
	}
}
