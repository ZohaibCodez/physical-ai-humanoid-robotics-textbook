# Quick Start: Deploy Backend to Railway

## 🎯 The Problem
Railway tries to deploy the entire repository, but you only want to deploy the `backend` folder.

## ✅ The Solution (3 Easy Steps)

### Step 1: Create Project
```
https://railway.app
↓
"New Project" → "Deploy from GitHub repo"
↓
Select: physical-ai-humanoid-robotics-textbook
```

### Step 2: Set Root Directory
```
Click your service
↓
Settings tab
↓
Source section → Root Directory: backend
↓
Save
```

### Step 3: Add Environment Variables
```
Variables tab → Add:
- GEMINI_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- QDRANT_COLLECTION
- CORS_ORIGINS
```

## 📦 What's Included

Your `backend/` folder already has:

✅ `railway.json` - Tells Railway how to build
✅ `nixpacks.toml` - Tells Railway how to run
✅ `requirements.txt` - Python dependencies

Railway automatically finds and uses these files!

## 🚀 Deployment Flow

```
Push to GitHub
    ↓
Railway detects changes
    ↓
Uses Root Directory: backend
    ↓
Reads railway.json & nixpacks.toml
    ↓
Installs Python dependencies
    ↓
Starts: uvicorn app.main:app
    ↓
✅ Live at: https://your-app.railway.app
```

## 🎉 Success!

Your backend is live when you see:

- ✅ Build completes successfully
- ✅ Health check: `https://your-app.railway.app/health` returns 200
- ✅ No errors in Railway logs

## 📖 More Details

- Full guide: `DEPLOYMENT.md`
- Monorepo setup: `RAILWAY-MONOREPO.md`
- Backend config: `backend/README.md`

---

**Total Time:** ~10 minutes from signup to live API! 🎊
