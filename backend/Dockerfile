# ==========================================
# LIGHTWEIGHT DOCKER IMAGE FOR FREE-TIER DEPLOYMENT
# Optimized for Render/Railway (512MB RAM)
# Final image: ~300MB (base + deps)
# ==========================================

# Stage 1: Builder (minimal build deps)
FROM python:3.11-slim as builder

WORKDIR /build

# Install ONLY essential build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Copy requirements
COPY requirements.txt .

# Install dependencies with aggressive size optimization
RUN pip install --no-cache-dir --user -r requirements.txt \
    && find /root/.local -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true \
    && find /root/.local -type f -name "*.pyc" -delete \
    && find /root/.local -type f -name "*.pyo" -delete

# Stage 2: Runtime (ultra-minimal)
FROM python:3.11-slim

WORKDIR /app

# Copy only Python dependencies from builder
COPY --from=builder /root/.local /root/.local

# Copy application code (no tests, no dev files)
COPY ./app ./app
COPY ./scripts ./scripts

# Environment optimizations for small footprint
ENV PATH=/root/.local/bin:$PATH \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# Expose port
EXPOSE 8000

# Lightweight health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import httpx; httpx.get('http://localhost:8000/v1/health', timeout=5)"

# Run with single worker (optimal for free tier 512MB RAM)
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "1"]
