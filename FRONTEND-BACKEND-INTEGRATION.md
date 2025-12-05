# Frontend-Backend Integration Guide

## 🎯 Connecting GitHub Pages Frontend to Backend

Your frontend is deployed on GitHub Pages, and you need to connect it to your backend API.

### 📍 Current Setup

**Frontend**: `https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/`  
**Backend**: Deploy to one of these platforms:
- Vercel: `https://your-project.vercel.app`
- Render: `https://your-project.onrender.com`
- Railway: `https://your-project.up.railway.app`

### 🔧 Step 1: Deploy Backend

Choose one platform and deploy:

#### Option A: Vercel (Recommended for serverless)
```bash
cd backend
vercel --prod
```

#### Option B: Render (Free tier Docker)
1. Go to https://dashboard.render.com/
2. Click "New +" → "Web Service"
3. Connect your GitHub repo
4. Configure:
   - **Name**: `physical-ai-backend`
   - **Environment**: `Docker`
   - **Dockerfile Path**: `backend/Dockerfile`
   - **Plan**: Free
5. Add environment variables (see below)
6. Click "Create Web Service"

#### Option C: Railway (Free credits)
```bash
cd backend
railway up
```

### 🔐 Step 2: Set Environment Variables

On your deployment platform, add these:

```bash
# Required
GOOGLE_API_KEY=<your-google-ai-studio-key>
QDRANT_URL=<your-qdrant-cloud-url>
QDRANT_API_KEY=<your-qdrant-api-key>
NEON_DATABASE_URL=<your-neon-postgres-url>

# CORS (important!)
CORS_ORIGINS=https://zohaibcodez.github.io,http://localhost:3000

# Optional
LITELLM_MODEL=gemini-1.5-flash
ENVIRONMENT=production
LOG_LEVEL=INFO
RATE_LIMIT_PER_MINUTE=10
RATE_LIMIT_PER_HOUR=50
```

**⚠️ CRITICAL**: The `CORS_ORIGINS` must include your GitHub Pages URL!

### 🔗 Step 3: Update Frontend Configuration

After deploying the backend, get your backend URL and update the frontend:

**File**: `src/theme/Layout/index.tsx`

```tsx
const backendUrl = 'https://your-actual-backend-url.vercel.app';
```

Replace `'https://your-actual-backend-url.vercel.app'` with your real backend URL.

### 🧪 Step 4: Test Locally First

Before deploying frontend changes:

1. **Start backend locally**:
```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

2. **Run integration test**:
```bash
python scripts/test_backend_integration.py
```

Expected output:
```
✅ All tests passed! Backend is working correctly.
   - Google Generative AI embeddings: ✅
   - Vector database search: ✅
   - API endpoints: ✅
```

3. **Test frontend locally**:
```bash
npm start
```

Open http://localhost:3000 and test the chatbot.

### 📤 Step 5: Deploy Frontend Changes

Once the backend URL is configured:

```bash
# Build
npm run build

# Deploy to GitHub Pages
npm run deploy
```

Or if using Git workflow:
```bash
git add src/theme/Layout/index.tsx
git commit -m "feat: Connect frontend to deployed backend"
git push origin main
```

GitHub Actions will auto-deploy to GitHub Pages.

### ✅ Step 6: Verify Integration

Visit your deployed site:
```
https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/
```

**Test the chatbot**:
1. Click the chat icon (bottom right)
2. Ask: "What is ROS2?"
3. Check if you get a response

### 🐛 Troubleshooting

#### CORS Error
```
Access to fetch at 'https://backend.com' from origin 'https://zohaibcodez.github.io' 
has been blocked by CORS policy
```

**Fix**: Update `CORS_ORIGINS` environment variable on your backend:
```bash
CORS_ORIGINS=https://zohaibcodez.github.io,http://localhost:3000
```

#### Connection Error
```
Cannot connect to server. Please check if the backend is running.
```

**Checks**:
1. Backend is deployed and running?
2. Backend URL in `src/theme/Layout/index.tsx` is correct?
3. HTTPS (not HTTP) for production?

#### 404 Not Found
```
POST https://backend.com/v1/chat/ask 404
```

**Check**: Backend routes are correct. Visit `https://your-backend.com/v1/docs` to see API docs.

#### No Response / Timeout
```
Request timeout after 30s
```

**Possible causes**:
1. Backend cold start (first request takes longer)
2. Google API rate limit
3. Qdrant connection issue

**Check backend logs** on your deployment platform.

### 🎨 Architecture Overview

```
User Browser (GitHub Pages)
        ↓
    HTTPS Request
        ↓
Backend API (Vercel/Render/Railway)
        ↓
    ┌─────────────┬─────────────┬──────────────┐
    ↓             ↓             ↓              ↓
Google AI    Qdrant Cloud  Neon Postgres  Gemini LLM
(Embeddings)  (Vectors)    (History)      (Answers)
```

**All lightweight, cloud-based, no local models!**

### 📊 Performance Expectations

| Metric | Value |
|--------|-------|
| Backend size | ~100-150MB |
| Cold start | 2-3 seconds |
| Response time | 2-5 seconds |
| RAM usage | ~200MB |
| Cost | **FREE** on all platforms |

### 🎯 Production Checklist

- [ ] Backend deployed on Vercel/Render/Railway
- [ ] Environment variables configured (especially CORS_ORIGINS)
- [ ] Backend URL updated in `src/theme/Layout/index.tsx`
- [ ] Integration test passed locally
- [ ] Frontend rebuilt and deployed
- [ ] Chatbot tested on production site
- [ ] HTTPS enabled (automatic on all platforms)
- [ ] API keys secured (not in frontend code)

### 🔒 Security Notes

1. **Never** put API keys in frontend code
2. **Always** use environment variables on backend
3. **Always** configure CORS properly
4. **Use** HTTPS in production (automatic)
5. **Monitor** rate limits on free tiers

### 📚 Additional Resources

- [Vercel Deployment](https://vercel.com/docs/deployments/overview)
- [Render Deployment](https://render.com/docs/deploy-fastapi)
- [GitHub Pages](https://docs.github.com/en/pages)
- [CORS Guide](https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS)

## 🚀 Quick Deploy Commands

### Deploy Backend to Vercel:
```bash
cd backend
vercel --prod
# Copy the deployment URL
```

### Update Frontend:
```bash
# Edit src/theme/Layout/index.tsx with backend URL
npm run build
npm run deploy
```

### Test Everything:
```bash
# Visit your site
https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/

# Test chatbot
# Click chat icon → Ask question → Get response
```

That's it! Your lightweight, cloud-only backend is now connected to your GitHub Pages frontend! 🎉
