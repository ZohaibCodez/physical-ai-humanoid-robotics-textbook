# 🚀 Deployment Guide - RAG Chatbot

Complete guide to deploy your Physical AI & Humanoid Robotics chatbot for FREE.

## 📋 Quick Overview

**Backend**: Railway.app (FREE $5/month credit)  
**Frontend**: GitHub Pages (FREE)  
**Vector DB**: Qdrant Cloud (FREE 1GB)  
**Database**: Neon Postgres (FREE 0.5GB)  
**LLM**: Google Gemini (FREE 1500 requests/day)

**Total Cost**: $0 per month ✅

---

## 🎯 Step-by-Step Deployment

### Phase 1: Get Your API Keys (10 minutes)

#### 1. Google Gemini API Key
```bash
1. Go to: https://aistudio.google.com/app/apikey
2. Click "Create API Key"
3. Copy the key (starts with AIza...)
4. Save it - you'll need it for Railway
```

#### 2. Qdrant Cloud Setup
```bash
1. Go to: https://cloud.qdrant.io
2. Sign up (FREE)
3. Create a cluster:
   - Name: robotics-textbook
   - Region: Choose closest to you (e.g., us-east, eu-west)
   - Plan: FREE tier (1GB storage)
4. Copy these values:
   - Cluster URL: https://xxxxx.gcp.cloud.qdrant.io
   - API Key: (from cluster settings)
```

#### 3. Neon Postgres (Optional but Recommended)
```bash
1. Go to: https://neon.tech
2. Sign up (FREE)
3. Create a project:
   - Name: rag-chatbot-db
   - Region: Same as Qdrant
4. Copy connection string from dashboard:
   postgresql://user:pass@host.neon.tech/dbname?sslmode=require
```

---

### Phase 2: Deploy Backend to Railway (15 minutes)

#### 1. Sign Up for Railway
```bash
1. Go to: https://railway.app
2. Click "Start a New Project"
3. Sign up with GitHub (FREE - no credit card needed)
4. You get $5/month credit (enough for this app!)
```

#### 2. Deploy from GitHub
```bash
1. In Railway dashboard, click "New Project"
2. Select "Deploy from GitHub repo"
3. Authorize Railway to access your repositories
4. Select: physical-ai-humanoid-robotics-textbook
5. Select branch: 002-rag-chatbot
```

#### 3. Configure Backend
```bash
1. After project is created, click on the service
2. Go to "Settings" tab
3. Set Root Directory: backend
   (This tells Railway to only deploy the backend folder)
4. Set Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

#### 4. Add Environment Variables
```bash
Click "Variables" tab and add these:

# Required Variables
GEMINI_API_KEY=AIza...your_key_here
QDRANT_URL=https://xxxxx.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key
QDRANT_COLLECTION=robotics_textbook_embeddings

# Optional (for conversation history)
NEON_DATABASE_URL=postgresql://user:pass@host.neon.tech/db

# CORS - IMPORTANT! Add your GitHub Pages URL
CORS_ORIGINS=https://YOUR_USERNAME.github.io,http://localhost:3000

# API Settings
LOG_LEVEL=INFO
PORT=8000
```

#### 5. Deploy & Get Your URL
```bash
1. Railway will automatically deploy (takes 2-3 minutes)
2. Once deployed, click "Settings" → "Generate Domain"
3. Your backend URL will be: https://your-app.up.railway.app
4. Copy this URL - you'll need it for frontend!
```

#### 6. Test Your Backend
```bash
# Test health endpoint
curl https://your-app.up.railway.app/v1/health

# Should return:
{
  "status": "healthy",
  "gemini_api": "connected",
  "qdrant": "connected",
  "postgres": "connected"
}
```

---

### Phase 3: Index Your Textbook (5 minutes)

You need to populate Qdrant with your textbook content.

**Option 1: Run Locally (Recommended)**
```bash
# In your local backend directory
cd backend

# Make sure virtual environment is activated
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Set environment variables temporarily
# Windows (PowerShell):
$env:GEMINI_API_KEY="your_key"
$env:QDRANT_URL="https://xxxxx.gcp.cloud.qdrant.io"
$env:QDRANT_API_KEY="your_key"
$env:QDRANT_COLLECTION="robotics_textbook_embeddings"

# macOS/Linux:
export GEMINI_API_KEY="your_key"
export QDRANT_URL="https://xxxxx.gcp.cloud.qdrant.io"
export QDRANT_API_KEY="your_key"
export QDRANT_COLLECTION="robotics_textbook_embeddings"

# Run indexing script
python scripts/index_textbook.py

# Expected output:
# ✅ Indexed 240 chunks from docs/
# ✅ Collection: robotics_textbook_embeddings ready
```

**Option 2: Run on Railway**
```bash
1. In Railway dashboard, click on your service
2. Go to "Settings" → "Deploy"
3. Add a one-time command:
   python scripts/index_textbook.py
4. This will run once and populate Qdrant
```

---

### Phase 4: Deploy Frontend to GitHub Pages (10 minutes)

#### 1. Update Frontend Configuration

Create `.env` file in root directory:
```bash
# In root of repository (not backend folder)
REACT_APP_CHAT_API_URL=https://your-app.up.railway.app
```

#### 2. Update Docusaurus Config

Edit `docusaurus.config.js`:
```javascript
const config = {
  // ... existing config ...
  
  customFields: {
    chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000',
  },
  
  // Make sure GitHub Pages settings are correct
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',
  organizationName: 'YOUR_USERNAME',
  projectName: 'physical-ai-humanoid-robotics-textbook',
};
```

#### 3. Update ChatWidget to Use Config

Edit `src/components/ChatWidget/index.tsx`:
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const ChatWidget = ({ apiUrl }) => {
  const { siteConfig } = useDocusaurusContext();
  const API_URL = apiUrl || siteConfig.customFields?.chatApiUrl || 'http://localhost:8000';
  // ... rest of component
```

#### 4. Build and Deploy

```bash
# Build the site
npm run build

# Deploy to GitHub Pages
GIT_USER=YOUR_USERNAME npm run deploy

# Or if using SSH:
USE_SSH=true npm run deploy
```

#### 5. Verify Deployment
```bash
1. Go to: https://YOUR_USERNAME.github.io/physical-ai-humanoid-robotics-textbook/
2. Chat widget should appear in bottom-right corner
3. Click it and test with: "What are the key components of ROS2?"
```

---

### Phase 5: Final Configuration (5 minutes)

#### Update Backend CORS

Make sure your Railway backend allows requests from GitHub Pages:

```bash
1. Go to Railway dashboard
2. Click on your service → "Variables"
3. Update CORS_ORIGINS:
   CORS_ORIGINS=https://YOUR_USERNAME.github.io,http://localhost:3000
4. Service will auto-restart with new CORS settings
```

---

## ✅ Deployment Checklist

Use this checklist to verify everything is working:

### Backend
- [ ] Railway service is running (green status)
- [ ] `/v1/health` endpoint returns healthy
- [ ] Environment variables are set correctly
- [ ] Qdrant has ~240 indexed documents
- [ ] CORS includes your GitHub Pages URL

### Frontend
- [ ] GitHub Pages site is live
- [ ] Chat widget appears on pages
- [ ] Chat opens when clicked
- [ ] Questions get responses (test with sample question)
- [ ] Citations are clickable and navigate correctly

### API Keys
- [ ] Google Gemini API key is valid (1500 req/day limit)
- [ ] Qdrant cluster is accessible
- [ ] Neon Postgres (optional) is connected

---

## 🔧 Testing Your Deployment

### Test Backend API
```bash
# Test health
curl https://your-app.up.railway.app/v1/health

# Test question endpoint
curl -X POST https://your-app.up.railway.app/v1/chat/ask/stream \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test_session",
    "question_text": "What are the key components of ROS2?",
    "context_mode": "full"
  }'
```

### Test Frontend Integration
```bash
1. Open your GitHub Pages site
2. Open browser DevTools (F12)
3. Click chat widget
4. Send a test question
5. Check Network tab for API calls
6. Check Console for any errors
```

---

## 🐛 Common Issues & Solutions

### Issue: "CORS Error" in Browser Console
**Solution:**
```bash
1. Go to Railway → Variables
2. Add your GitHub Pages URL to CORS_ORIGINS:
   CORS_ORIGINS=https://yourusername.github.io,http://localhost:3000
3. Wait 30 seconds for restart
```

### Issue: "Connection Refused" from Frontend
**Solution:**
```bash
1. Check .env has correct REACT_APP_CHAT_API_URL
2. Verify Railway service is running (green status)
3. Test backend health: curl https://your-app.up.railway.app/v1/health
```

### Issue: "Qdrant returns 0 results"
**Solution:**
```bash
# Re-run indexing script
python scripts/index_textbook.py

# Verify in Qdrant dashboard:
# Collection: robotics_textbook_embeddings
# Points: ~240
```

### Issue: "Rate Limit Exceeded"
**Solution:**
```bash
# Google Gemini free tier: 1500 requests/day
# Wait 24 hours or upgrade to paid tier
# Fallback to local embeddings is automatic
```

---

## 📊 Monitoring Your Deployment

### Railway Logs
```bash
1. Railway dashboard → Your service
2. Click "Logs" tab
3. Monitor requests, errors, response times
```

### Qdrant Dashboard
```bash
1. Qdrant Cloud dashboard
2. Check collection size
3. Monitor query performance
```

### GitHub Pages
```bash
1. Repository → Settings → Pages
2. Check build status
3. View deployment history
```

---

## 💰 Cost Breakdown (FREE!)

| Service | Free Tier | Usage |
|---------|-----------|-------|
| Railway | $5/month credit | ~$2-3/month actual |
| Qdrant Cloud | 1GB storage | ~50MB used |
| Neon Postgres | 0.5GB storage | ~10MB used |
| Google Gemini | 1500 req/day | Enough for testing |
| GitHub Pages | Unlimited | Free for public repos |
| **Total** | **$0/month** | ✅ Fully FREE |

---

## 🚀 Next Steps

After successful deployment:

1. **Custom Domain** (Optional):
   - Railway: Add custom domain in settings
   - GitHub Pages: Configure CNAME

2. **Monitoring**:
   - Set up Railway alerts for downtime
   - Monitor Gemini API quota usage

3. **Performance**:
   - Add caching for frequent questions
   - Optimize embedding search parameters

4. **Production Readiness**:
   - Add error tracking (Sentry)
   - Set up uptime monitoring (UptimeRobot)
   - Implement analytics (Google Analytics)

---

## 📚 Additional Resources

- [Railway Documentation](https://docs.railway.app)
- [Qdrant Cloud Docs](https://qdrant.tech/documentation)
- [Neon Postgres Docs](https://neon.tech/docs)
- [Google AI Studio](https://ai.google.dev)
- [GitHub Pages Guide](https://docs.github.com/pages)

---

## 💬 Need Help?

- Backend issues: Check Railway logs
- Frontend issues: Check browser console
- API issues: Test with curl/Postman
- Deployment questions: Review this guide step-by-step

**Your chatbot is now live! 🎉**
