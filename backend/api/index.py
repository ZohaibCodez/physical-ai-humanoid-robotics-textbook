from app.main import app

# Vercel serverless function handler
def handler(request):
    return app
