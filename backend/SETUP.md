# Authentication Setup Guide

## Environment Configuration

### 1. Create .env file

Copy the example environment file:

```bash
cd backend
cp .env.example .env
```

### 2. Configure Required Environment Variables

Edit `backend/.env` and set the following variables:

#### Required for Authentication:

```dotenv
# Neon Postgres Database (REQUIRED)
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# JWT Authentication (REQUIRED)
JWT_SECRET_KEY=your_jwt_secret_key_min_32_characters_long_change_in_production
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=30
```

**Important**: 
- `NEON_DATABASE_URL`: Get this from your Neon Postgres dashboard
- `JWT_SECRET_KEY`: Generate a secure random string (minimum 32 characters)
  - Example generation: `python -c "import secrets; print(secrets.token_urlsafe(32))"`

#### Other Required Variables:

```dotenv
# API Keys
GOOGLE_API_KEY=your_google_api_key_here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=textbook_chunks

# CORS Origins
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

### 3. Install Dependencies

Install Python dependencies in virtual environment:

```bash
cd backend
uv venv
uv pip install -r requirements.txt
```

### 4. Run Database Migration

Execute the authentication tables migration:

```bash
# From backend directory
python scripts/run_migration.py
```

Expected output:
```
🔗 Connecting to Neon Postgres...
📝 Executing migration: create_auth_tables.sql
✅ Verifying tables...
✅ Migration successful! Tables created:
   - users
   - user_preferences
```

### 5. Start Backend Server

```bash
# From backend directory
uvicorn app.main:app --reload --port 8000
```

### 6. Start Frontend Development Server

```bash
# From project root
npm start
```

### 7. Test Authentication Flow

1. Navigate to `http://localhost:3000/signup`
2. Fill in signup form with:
   - Email address
   - Password (min 8 characters)
   - Full name
   - Programming experience (beginner/intermediate/advanced)
   - Hardware access (cloud_only/basic/full_lab)
   - Preferred language (en/ur/both)
3. Submit form
4. Verify redirect to home page with authenticated session

## Troubleshooting

### Database Connection Issues

**Error**: `NEON_DATABASE_URL environment variable not set`
- **Solution**: Ensure `.env` file exists in `backend/` directory and contains valid `NEON_DATABASE_URL`

**Error**: `Failed to connect to Neon Postgres`
- **Solution**: 
  - Verify database URL format: `postgresql://user:password@host.neon.tech/dbname?sslmode=require`
  - Check Neon dashboard for correct connection string
  - Ensure IP allowlist includes your IP (or set to allow all for development)

### JWT Configuration Issues

**Error**: `JWT_SECRET_KEY is required`
- **Solution**: Add `JWT_SECRET_KEY` to `.env` file with minimum 32 characters

**Warning**: Using default JWT_SECRET_KEY in production
- **Solution**: Generate secure key: `python -c "import secrets; print(secrets.token_urlsafe(32))"`

### Migration Issues

**Error**: `Migration file not found`
- **Solution**: Run migration script from backend directory: `cd backend && python scripts/run_migration.py`

**Error**: `Table already exists`
- **Solution**: Tables already created. Migration is idempotent, safe to run multiple times.

### Frontend API Connection Issues

**Error**: `Failed to fetch from http://localhost:8000`
- **Solution**: 
  - Ensure backend server is running on port 8000
  - Check `CORS_ORIGINS` in `.env` includes `http://localhost:3000`
  - Verify `REACT_APP_API_URL` environment variable (defaults to `http://localhost:8000`)

## Security Best Practices

### Development
- ✅ Use `.env` file for local development
- ✅ Never commit `.env` to git (already in `.gitignore`)
- ✅ Use strong JWT secret (32+ characters)
- ✅ Keep `ACCESS_TOKEN_EXPIRE_MINUTES` short (30 minutes recommended)

### Production
- ✅ Use environment variables (not `.env` files)
- ✅ Generate cryptographically secure JWT_SECRET_KEY
- ✅ Enable `secure=True` for cookies (HTTPS only)
- ✅ Set appropriate `CORS_ORIGINS` (no wildcards)
- ✅ Use Neon connection pooling for production workloads
- ✅ Enable database SSL mode (`sslmode=require`)

## Environment Variables Reference

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `NEON_DATABASE_URL` | ✅ Yes | - | Neon Postgres connection string |
| `JWT_SECRET_KEY` | ✅ Yes | - | Secret key for JWT signing (min 32 chars) |
| `JWT_ALGORITHM` | No | `HS256` | JWT signing algorithm |
| `ACCESS_TOKEN_EXPIRE_MINUTES` | No | `30` | Access token lifetime in minutes |
| `REFRESH_TOKEN_EXPIRE_DAYS` | No | `30` | Refresh token lifetime in days |
| `CORS_ORIGINS` | Yes | `http://localhost:3000,http://localhost:8000` | Comma-separated allowed origins |
| `GOOGLE_API_KEY` | Yes | - | Google Gemini API key |
| `QDRANT_URL` | Yes | - | Qdrant vector database URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
