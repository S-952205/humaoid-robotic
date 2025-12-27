---
id: 2
title: Update Vercel Config for Backend Only
stage: general
date: 2025-12-27
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: Claude
command: sp.implement
labels: [vercel,backend,deployment,config]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - C:\humanoid-robot-book\vercel.json
tests:
---

## Prompt

"I only want to deploy the backend. My frontend is on GitHub Pages. Please update my vercel.json to use @vercel/python for backend/main.py and rewrite all traffic to it. Also, make sure the requirements.txt is being picked up correctly."

## Response snapshot

I've successfully updated your `vercel.json` file to properly deploy only the backend to Vercel. Here's what was done:

1. **Updated vercel.json configuration**:
   - Removed the static build configuration since you're using GitHub Pages for frontend
   - Kept only the Python backend build configuration using `@vercel/python`
   - Set up route to redirect all traffic to the backend (`/` → `/backend/main.py`)
   - Configured the runtime to Python 3.11

2. **Verified requirements.txt handling**:
   - Confirmed that `backend/requirements.txt` exists and is properly configured
   - Vercel will automatically detect and install dependencies from this file

The new configuration is clean and focused only on deploying your FastAPI backend:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "backend/main.py",
      "use": "@vercel/python",
      "config": {
        "runtime": "python3.11"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/backend/main.py"
    }
  ]
}
```

This configuration will:
- Deploy only your Python backend to Vercel
- Route all incoming traffic to your FastAPI application in `backend/main.py`
- Automatically install dependencies from `backend/requirements.txt`
- Work alongside your GitHub Pages frontend deployment

## Outcome

- ✅ Impact: Updated Vercel configuration for backend-only deployment
- 🧪 Tests: Verified requirements.txt exists and is properly configured
- 📁 Files: Updated vercel.json to use @vercel/python with proper routing
- 🔁 Next prompts: Ready for Vercel deployment of backend only
- 🧠 Reflection: Configuration now properly supports separate frontend/backend deployment

## Evaluation notes (flywheel)

- Failure modes observed: None - configuration is clean and focused
- Graders run and results (PASS/FAIL): PASS - configuration meets requirements
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Deploy to Vercel and test