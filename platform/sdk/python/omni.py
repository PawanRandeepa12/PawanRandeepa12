"""
OMNI Python SDK - Drop-in for OpenAI + fal.ai + x402
pip install openai requests
"""

import os
from openai import OpenAI

class OmniClient:
    def __init__(self, api_key: str = None, base_url: str = "https://api.omni.ai/v1"):
        self.api_key = api_key or os.getenv("OMNI_API_KEY") or "omni_demo_12345"
        self.base_url = base_url
        self.openai_client = OpenAI(api_key=self.api_key, base_url=base_url)
        # For local dev: http://localhost:8080/v1

    # LLM - OpenAI compat passthrough
    @property
    def chat(self):
        return self.openai_client.chat

    @property
    def completions(self):
        return self.openai_client.completions

    @property
    def embeddings(self):
        return self.openai_client.embeddings

    # Cost intelligence - OMNI extension
    def recommend_model(self, prompt_template: str, quality_threshold=0.9, cost_pref=0.7):
        import requests
        # Calls control plane /v1/cost/recommend
        control_url = self.base_url.replace("/v1", "").replace(":8080", ":8001") + "/v1/cost/recommend"
        try:
            resp = requests.post(control_url, json={
                "prompt_template": prompt_template,
                "quality_threshold": quality_threshold,
                "preferences": {"cost": cost_pref, "quality": 1-cost_pref}
            }, timeout=5)
            return resp.json()
        except:
            return {"recommended_model": "deepseek-v3", "savings_estimate": 77.6, "message": "Mock: DeepSeek 77% cheaper than GPT-5 with 91% parity"}

    # MCP registry
    def list_mcp_servers(self, category=None, search=None):
        import requests
        control_url = self.base_url.replace("/v1", "").replace(":8080", ":8001") + "/v1/mcp/registry"
        try:
            resp = requests.get(control_url, params={"category": category, "search": search}, timeout=5)
            return resp.json()
        except:
            return {"servers": [{"name": "github", "description": "Mock GitHub MCP"}, {"name": "brave-search", "price": 0.001}], "count": 2}

    # Commerce - x402 paid tool decorator (pattern from Vercel x402-mcp)
    def paid_tool(self, price: float, currency="USDC", chain="base"):
        def decorator(func):
            def wrapper(*args, **kwargs):
                # In real: check X-Payment-Receipt header verification via facilitator
                # For demo, just log
                print(f"[OMNI Commerce] Tool {func.__name__} priced at {price} {currency} on {chain} - would verify x402 receipt here")
                return func(*args, **kwargs)
            wrapper._omni_priced = True
            wrapper._omni_price = price
            return wrapper
        return decorator

# Example usage
if __name__ == "__main__":
    client = OmniClient(base_url="http://localhost:8080/v1") # local gateway

    # Auto router
    print("=== Chat with auto router ===")
    try:
        resp = client.chat.completions.create(
            model="auto",
            messages=[{"role": "user", "content": "Classify sentiment: I love OMNI"}],
            extra_body={"model_preferences": {"cost": 0.8, "quality": 0.2}}
        )
        print(resp.choices[0].message.content)
        print("Meta:", resp.model_dump().get('omni') if hasattr(resp, 'model_dump') else 'mock')
    except Exception as e:
        print(f"Gateway not running locally, mock: {e}")

    # Cost recommendation
    print("\n=== Cost Recommendation ===")
    rec = client.recommend_model("Classify sentiment of customer reviews", quality_threshold=0.88, cost_pref=0.8)
    print(rec)

    # MCP registry
    print("\n=== MCP Registry ===")
    registry = client.list_mcp_servers()
    print(f"Found {registry.get('count')} servers")

    # Monetized tool
    @client.paid_tool(price=0.001, currency="USDC", chain="base")
    def brave_search(query: str):
        return {"result": f"Search results for {query}"}

    print("\n=== Monetized Tool ===")
    print(brave_search("OMNI AI Gateway"))
