# Railway Monorepo Deployment Guide

## Problem
Railway deploys the entire repository by default. For a monorepo with frontend and backend, you need to deploy only the `backend` folder.

## ✅ Solution: 3 Methods

### Method 1: Railway Dashboard (Recommended - Easiest)

1. **Create New Project**
   - Go to https://railway.app
   - Click "New Project" → "Deploy from GitHub repo"
   - Select your repository: `physical-ai-humanoid-robotics-textbook`

2. **Configure Root Directory**
   - Click on your deployed service
   - Go to **Settings** tab
   - Scroll to **Source** section
   - Set **Root Directory**: `backend`
   - Click **Save**

3. **Deploy**
   - Railway will automatically redeploy with the correct root directory
   - Only files in `backend/` folder will be used

### Method 2: Configuration Files (Automatic)

The repository already includes these files in the `backend/` folder:

#### `backend/railway.json`
```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "pip install --no-cache-dir -r requirements.txt"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "healthcheckPath": "/health",
    "healthcheckTimeout": 100,
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

#### `backend/nixpacks.toml`
```toml
[phases.setup]
nixPkgs = ["python312", "postgresql"]

[phases.install]
cmds = [
  "pip install --upgrade pip",
  "pip install --no-cache-dir -r requirements.txt"
]

[start]
cmd = "uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}"
```

**How it works:**
- Railway automatically detects `railway.json` and `nixpacks.toml` in the root directory
- When you set Root Directory to `backend`, Railway uses these files
- No manual build/start commands needed

### Method 3: Environment Variable

Alternative method if you prefer environment variables:

1. Go to your Railway service
2. Click **Variables** tab
3. Add: `RAILWAY_ROOT_DIRECTORY=backend`
4. Railway will use this as the root directory

## 📋 Complete Deployment Checklist

- [ ] Repository connected to Railway
- [ ] Root Directory set to `backend` (Method 1 or 3)
- [ ] Configuration files present: `railway.json` and `nixpacks.toml`
- [ ] Environment variables added (see DEPLOYMENT.md)
- [ ] First deployment successful
- [ ] Health check working: `https://your-app.railway.app/health`
- [ ] Textbook indexed (run `python scripts/index_textbook.py`)

## 🔍 Verification

After deployment, check:

```bash
# Test health endpoint
curl https://your-app.railway.app/health

# Expected response:
{
  "status": "healthy",
  "gemini_configured": true,
  "qdrant_configured": true,
  "postgres_configured": true
}
```

## 🐛 Troubleshooting

### Issue: Railway deploys entire repo

**Solution:**
- Double-check Root Directory is set to `backend` in Settings → Source
- Redeploy after saving the setting

### Issue: Build fails with "No module named 'app'"

**Solution:**
- Ensure `railway.json` is in `backend/` folder
- Verify Root Directory is `backend`, not `backend/app`

### Issue: Environment variables not loading

**Solution:**
- Check Variables tab in Railway dashboard
- Verify all required variables are set (GEMINI_API_KEY, QDRANT_URL, etc.)
- Redeploy after adding variables

## 📚 References

- Railway Monorepo Docs: https://docs.railway.app/guides/monorepo
- Railway.json Schema: https://railway.app/railway.schema.json
- Nixpacks Documentation: https://nixpacks.com

## 🎉 Success Indicators

When properly configured, you'll see:

```
✅ Build logs show: "Using root directory: backend"
✅ Only backend files in build context
✅ Python dependencies installed correctly
✅ Uvicorn starts on $PORT
✅ Health endpoint returns 200 OK
```

---

**Need help?** Check the full deployment guide in `DEPLOYMENT.md` or backend setup in `backend/README.md`
