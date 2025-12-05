# RAG Chatbot - Setup & Deployment Guide

This guide covers setting up the RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics documentation site.

## Architecture Overview

```
┌─────────────────────────────────────────────────┐
│             FRONTEND (Docusaurus)               │
│  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ ChatBot     │  │ TextSelectionHandler    │  │
│  │ Component   │  │ (Ask about selection)   │  │
│  └──────┬──────┘  └───────────┬─────────────┘  │
│         │                     │                 │
│         └──────────┬──────────┘                 │
│                    ▼                            │
└────────────────────┼────────────────────────────┘
                     │ REST API
                     ▼
┌─────────────────────────────────────────────────┐
│              BACKEND (FastAPI)                  │
│  ┌─────────────────────────────────────────┐   │
│  │           RAG Service                    │   │
│  │  ┌─────────┐  ┌────────┐  ┌──────────┐ │   │
│  │  │Retriever│  │Generator│ │Embeddings│ │   │
│  │  └────┬────┘  └────┬────┘ └────┬─────┘ │   │
│  └───────┼────────────┼───────────┼───────┘   │
│          ▼            ▼           ▼            │
│    ┌──────────┐  ┌─────────┐  ┌──────────┐   │
│    │  Qdrant  │  │  Neon   │  │  OpenAI  │   │
│    │  Cloud   │  │Postgres │  │   API    │   │
│    └──────────┘  └─────────┘  └──────────┘   │
└─────────────────────────────────────────────────┘
```

## Prerequisites

- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- OpenAI API key
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)

## Step 1: Set Up External Services

### 1.1 Qdrant Cloud

1. Create account at https://cloud.qdrant.io/
2. Create a new cluster (free tier available)
3. Note your cluster URL and API key

### 1.2 Neon Postgres

1. Create account at https://neon.tech/
2. Create a new project
3. Get your connection string from the dashboard

### 1.3 OpenAI API

1. Get API key from https://platform.openai.com/
2. Ensure you have credits for embeddings and chat completions

## Step 2: Backend Setup

### 2.1 Install Dependencies

```bash
cd api
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2.2 Configure Environment

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.cloud
QDRANT_API_KEY=your-qdrant-api-key
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
CORS_ORIGINS=http://localhost:3000,https://aliraza4278.github.io
ENVIRONMENT=development
```

### 2.3 Initialize Database

The database tables are created automatically on first run, but you can verify:

```bash
python -c "from models.database import init_db; init_db()"
```

### 2.4 Ingest Documentation

Run the ingestion script to process all documentation:

```bash
python scripts/ingest_docs.py --force
```

This will:
- Load all markdown files from `docs/`
- Chunk them into ~400 token segments
- Generate embeddings using OpenAI
- Store in Qdrant vector database

### 2.5 Run Development Server

```bash
uvicorn main:app --reload --port 8000
```

API will be available at http://localhost:8000

- Swagger docs: http://localhost:8000/docs
- Health check: http://localhost:8000/api/health

## Step 3: Frontend Setup

### 3.1 Install Dependencies

```bash
npm install
```

### 3.2 Configure API URL

Edit `src/theme/Root.tsx` to set your API URL:

```tsx
const API_URL = process.env.REACT_APP_CHATBOT_API_URL ||
  (typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://your-api-deployment.render.com');  // Update this!
```

### 3.3 Run Development Server

```bash
npm start
```

Site will be available at http://localhost:3000

## Step 4: Deploy Backend

### Option A: Render (Recommended for Free Tier)

1. Push code to GitHub
2. Create new Web Service at https://render.com
3. Connect your repository
4. Set root directory to `api/`
5. Add environment variables in Render dashboard
6. Deploy!

### Option B: Vercel

1. Install Vercel CLI: `npm i -g vercel`
2. From `api/` directory: `vercel`
3. Set environment variables in Vercel dashboard

### Option C: Railway

1. Create account at https://railway.app/
2. New Project → Deploy from GitHub
3. Configure environment variables

## Step 5: Deploy Frontend

The frontend automatically deploys to GitHub Pages via the existing workflow.

After deploying the backend:

1. Update `API_URL` in `src/theme/Root.tsx` with your deployed API URL
2. Commit and push changes
3. GitHub Actions will rebuild and deploy

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/chat` | POST | Send message, get RAG response |
| `/api/conversations/{id}/messages` | GET | Get conversation history |
| `/api/ingest` | POST | Trigger document ingestion |
| `/api/ingest/status` | GET | Check ingestion status |
| `/api/health` | GET | Health check |

### Example Chat Request

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is a ROS 2 node?",
    "session_id": "test-session-123"
  }'
```

## Features

### Text Selection Q&A

1. Select any text in the documentation
2. Click "Ask about this" tooltip
3. Chat opens with context pre-filled
4. Ask questions about the selected text

### Conversation History

- Conversations are stored in Neon Postgres
- History persists across browser sessions
- Click "New Chat" to start fresh

### Source Citations

- Every response includes source references
- Click sources to navigate to relevant sections
- Relevance scores shown for each source

## Troubleshooting

### Chatbot not appearing

1. Check browser console for errors
2. Verify `src/theme/Root.tsx` is loading
3. Check CSS is being applied

### API connection errors

1. Verify backend is running
2. Check CORS configuration matches frontend origin
3. Test health endpoint: `curl http://localhost:8000/api/health`

### Poor retrieval results

1. Re-run ingestion: `python scripts/ingest_docs.py --force`
2. Check chunk sizes in `config.py`
3. Verify all docs are being processed

### OpenAI rate limits

1. Implement request throttling on frontend
2. Use smaller batch sizes for ingestion
3. Consider using OpenAI usage tiers

## Cost Considerations

| Service | Free Tier Limits |
|---------|------------------|
| Qdrant Cloud | 1GB storage, 1M vectors |
| Neon Postgres | 0.5GB storage, 3GB transfer |
| OpenAI | Pay per token (~$0.02/1K tokens for embeddings) |
| Render/Vercel | Limited compute hours |

## Security Notes

- Never commit `.env` files
- Use environment variables for all secrets
- Enable HTTPS in production
- Validate and sanitize user inputs
- Rate limit API endpoints
