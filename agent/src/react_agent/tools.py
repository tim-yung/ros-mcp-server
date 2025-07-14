"""This module provides example tools for web scraping and search functionality.

It loads mcp_config.json to load the mcp server.

These tools are intended as free examples to get started. For production use,
consider implementing more robust and specialized tools tailored to your needs.
"""

import asyncio
from typing import Any, Callable, List
from langchain_mcp_adapters.client import MultiServerMCPClient

from react_agent.configuration import Configuration
import json

try:
    with open("src/react_agent/mcp_config.json") as f:
        client = MultiServerMCPClient(json.loads(f.read()))
except FileNotFoundError:
    # Fallback to empty tools list if config file is not found
    client = None

# Retrieve tools synchronously at import time with graceful degradation
TOOLS: List[Callable[..., Any]] = asyncio.run(client.get_tools())

print(f"Loaded {len(TOOLS)} tools.")
