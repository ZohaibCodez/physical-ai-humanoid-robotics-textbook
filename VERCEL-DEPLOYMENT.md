# Deploy Backend to Vercel

## ✅ Why Vercel Instead of Railway?

- ✅ **No image size limit** - Railway's 4GB limit is too small
- ✅ **Generous free tier** - Serverless functions scale automatically
- ✅ **Better for FastAPI** - Native Python support
- ✅ **Easy deployment** - Deploy in 2 minutes

## 🚀 Deployment Steps

### 1. Install Vercel CLI (Optional)

```bash
npm install -g vercel
```

### 2. Deploy via Vercel Dashboard (Recommended)

1. **Go to Vercel:**
   - Visit: https://vercel.com
   - Sign up with GitHub (FREE)

2. **Import Project:**
   - Click "Add New" → "Project"
   - Import your GitHub repository: `physical-ai-humanoid-robotics-textbook`

3. **Configure Project:**
   ```
   Framework Preset: Other
   Root Directory: backend
   Build Command: (leave empty)
   Output Directory: (leave empty)
   Install Command: pip install -r requirements.txt
   ```

4. **Add Environment Variables:**
   Click "Environment Variables" and add:
   ```env
   GOOGLE_API_KEY=your_google_api_key_here
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=robotics_textbook_embeddings
   NEON_DATABASE_URL=your_neon_database_url_here
   CORS_ORIGINS=https://yourusername.github.io,http://localhost:3000
   ENVIRONMENT=production
   LOG_LEVEL=INFO
   OPENAI_DEFAULT_MODEL=gemini-2.0-flash
   LITELLM_MODEL=gemini-2.0-flash
   ```

5. **Deploy:**
   - Click "Deploy"
   - Wait 1-2 minutes
   - You'll get a URL like: `https://your-project.vercel.app`

### 3. Alternative: Deploy via CLI

```bash
cd backend
vercel
# Follow prompts:
# - Set root directory: backend
# - Add environment variables when asked
```

## 📝 Important Notes

### Vercel Configuration

The `backend/vercel.json` file is already configured:
```json
{
  "version": 2,
  "builds": [
    {
      "src": "app/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "app/main.py"
    }
  ]
}
```

### CORS Setup

After deployment, update your frontend's API URL:

**In `docusaurus.config.js`:**
```javascript
customFields: {
  chatApiUrl: 'https://your-project.vercel.app'
}
```

**Or in `.env`:**
```env
REACT_APP_CHAT_API_URL=https://your-project.vercel.app
```

### Indexing Textbook

After first deployment, run the indexing script locally:

```bash
cd backend
# Make sure your .env points to Vercel's URL for testing
python scripts/index_textbook.py
```

Or run it via Vercel CLI:
```bash
vercel env pull .env.production
python scripts/index_textbook.py
```

## 🎯 Verify Deployment

Test your endpoints:

```bash
# Health check
curl https://your-project.vercel.app/health

# Expected response:
{
  "status": "healthy",
  "gemini_configured": true,
  "qdrant_configured": true,
  "postgres_configured": true
}
```

## 🐛 Troubleshooting

### Issue: "Module not found"

**Solution:** Ensure `requirements.txt` includes all dependencies:
```bash
pip freeze > requirements.txt
```

### Issue: "Function timeout"

**Solution:** Vercel free tier has 10s timeout. For long-running tasks:
- Use Vercel Pro ($20/month) with 60s timeout
- Or use background jobs/queues

### Issue: "Cold starts"

**Solution:** Vercel serverless functions have cold starts (~1-2s). This is normal. First request might be slow.

## 💰 Cost

- ✅ **Free Tier:** 100GB bandwidth, 100GB-hours compute/month
- ✅ **More than enough** for your textbook chatbot
- ✅ **No credit card required** for free tier

## 🎉 Success!

Once deployed:
1. ✅ Backend live at: `https://your-project.vercel.app`
2. ✅ Auto-deploys on every `git push`
3. ✅ Free HTTPS included
4. ✅ CDN and global edge network

**Update your frontend with the new Vercel URL and you're done!** 🚀
