# OAuth Setup Guide

This guide explains how to set up Google and GitHub OAuth for the Physical AI & Humanoid Robotics Course Book.

## Prerequisites

- Node.js 16+ installed
- npm or yarn package manager
- Google and/or GitHub OAuth credentials

## Step 1: Set Up Google OAuth

### 1.1 Create Google OAuth Credentials

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Enable the Google+ API:
   - Go to "APIs & Services" > "Library"
   - Search for "Google+ API"
   - Click "Enable"
4. Create OAuth 2.0 credentials:
   - Go to "APIs & Services" > "Credentials"
   - Click "Create Credentials" > "OAuth client ID"
   - Choose "Web application"
   - Under "Authorized JavaScript origins", add: `http://localhost:3000`
   - Under "Authorized redirect URIs", add: `http://localhost:3000/humanoid-robot-book/auth/callback`
   - Click "Create"
5. Copy the **Client ID** from the created credentials

### 1.2 Add Google Credentials to Environment

Create a `.env.local` file in the project root:

```bash
REACT_APP_GOOGLE_CLIENT_ID=your_google_client_id_here
```

## Step 2: Set Up GitHub OAuth

### 2.1 Create GitHub OAuth App

1. Go to [GitHub Settings > Developer settings](https://github.com/settings/developers)
2. Click "OAuth Apps" > "New OAuth App"
3. Fill in the form:
   - **Application name**: Physical AI Robotics Course Book
   - **Homepage URL**: `http://localhost:3000/humanoid-robot-book`
   - **Authorization callback URL**: `http://localhost:3000/humanoid-robot-book/auth/callback`
4. Click "Register application"
5. Copy the **Client ID**

### 2.2 Add GitHub Credentials to Environment

Update `.env.local`:

```bash
REACT_APP_GITHUB_CLIENT_ID=your_github_client_id_here
```

## Step 3: Set Up Backend OAuth Handler

The backend server handles the OAuth token exchange. This is required for actual OAuth to work.

### 3.1 Install Backend Dependencies

```bash
npm install express cors axios dotenv
npm install -D ts-node typescript @types/express @types/node
```

### 3.2 Create Backend Environment File

Create a `.env` file in the project root:

```bash
# Backend OAuth Configuration
PORT=5000
FRONTEND_URL=http://localhost:3000

# Google OAuth (from Google Cloud Console)
GOOGLE_CLIENT_ID=your_google_client_id
GOOGLE_CLIENT_SECRET=your_google_client_secret

# GitHub OAuth (from GitHub Settings)
GITHUB_CLIENT_ID=your_github_client_id
GITHUB_CLIENT_SECRET=your_github_client_secret
```

### 3.3 Run the Backend Server

In a separate terminal:

```bash
npx ts-node server.ts
```

You should see:
```
OAuth server listening on http://localhost:5000
```

## Step 4: Start the Development Server

In another terminal (keeping the backend running):

```bash
npm run start
```

## Step 5: Test OAuth Login

1. Open http://localhost:3000/humanoid-robot-book/auth
2. Click "🔍 Google" or "🐙 GitHub" button
3. You should be redirected to the OAuth provider
4. Log in with your credentials
5. You'll be redirected back to the app with your profile loaded

## Production Setup

For production deployment:

1. Update the URLs in Google and GitHub OAuth settings to your production domain
2. Update environment variables with production credentials
3. Ensure the backend server is running on your production server
4. Use HTTPS instead of HTTP
5. Set proper CORS origins

## Troubleshooting

### "Google OAuth Client ID not configured"
- Make sure `.env.local` has `REACT_APP_GOOGLE_CLIENT_ID` set
- Restart the dev server after adding environment variables

### "Backend server may not be running"
- Check that `npm run server` is running on port 5000
- Verify `.env` file has correct backend credentials
- Check browser console for specific error messages

### "State mismatch" error
- This is a CSRF protection error. Try clearing cookies and retrying
- Make sure you're using the correct redirect URL

### OAuth buttons do nothing
- Check browser console for JavaScript errors
- Make sure environment variables are set
- Verify the backend server is running

## Security Considerations

- Never commit `.env` or `.env.local` files to version control
- Always use HTTPS in production
- Regenerate OAuth credentials if they're accidentally exposed
- Keep the client secrets in the backend only, never in frontend code
- Validate state parameter on callback to prevent CSRF attacks

## File Structure

```
project-root/
├── .env                    # Backend credentials (NEVER commit)
├── .env.local             # Frontend credentials (NEVER commit)
├── .env.local.example     # Template for frontend setup
├── server.ts              # Backend OAuth handler
├── src/
│   ├── pages/
│   │   ├── auth.tsx       # Login form
│   │   └── auth/
│   │       └── callback.tsx # OAuth callback handler
│   └── contexts/
│       └── AuthContext.tsx # Auth state management
└── ...
```

## Next Steps

1. Set up both Google and GitHub OAuth for a complete authentication system
2. Consider adding email verification
3. Implement user profile management
4. Add logout functionality to navbar
5. Create user dashboard for tracking course progress
