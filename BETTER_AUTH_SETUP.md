# Better Auth Setup Guide

This guide will help you set up full authentication with **better-auth** for the Physical AI & Humanoid Robotics Course Book with **working Google and GitHub OAuth**.

## Quick Start (5 minutes)

### 1. Install Dependencies

```bash
npm install
```

### 2. Create Environment File

Copy the example file:
```bash
cp .env.example .env
```

Add your values to `.env` (see below for how to get credentials).

### 3. Run the Servers

**Terminal 1 - Backend Auth Server:**
```bash
npm run server
# or with ts-node directly:
npx ts-node auth-server.ts
```

You should see:
```
✅ Better Auth server running on http://localhost:5000
📝 API Base: http://localhost:5000/api/auth
```

**Terminal 2 - Frontend:**
```bash
npm run start
```

Access at: `http://localhost:3000/humanoid-robot-book/`

### 4. Test Authentication

Visit: `http://localhost:3000/humanoid-robot-book/auth`

- Sign up with email/password
- Sign in with email/password
- Try Google login (if configured)
- Try GitHub login (if configured)

---

## Full Setup Instructions

### Prerequisites

- Node.js 18+ installed
- Git
- Google and GitHub developer accounts

### Step 1: Backend Server Setup

The authentication server is in `auth-server.ts` and uses:
- **Framework**: Express.js
- **Auth**: better-auth
- **Database**: SQLite (file: `data/auth.db`)

No additional setup needed - it works out of the box!

### Step 2: Google OAuth Setup

#### A. Create Google OAuth Credentials

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project (or use existing)
3. Enable Google+ API:
   - Go to "APIs & Services" > "Library"
   - Search "Google+ API"
   - Click "Enable"
4. Create OAuth 2.0 credentials:
   - Go to "APIs & Services" > "Credentials"
   - Click "Create Credentials" > "OAuth client ID"
   - Choose "Web application"
   - Under "Authorized JavaScript origins", add:
     - `http://localhost:5000`
   - Under "Authorized redirect URIs", add:
     - `http://localhost:5000/api/auth/google/callback`
   - Click "Create"
5. **Copy the Client ID and Client Secret**

#### B. Add to .env

In your `.env` file:
```bash
GOOGLE_CLIENT_ID=your_client_id_here
GOOGLE_CLIENT_SECRET=your_client_secret_here
```

### Step 3: GitHub OAuth Setup

#### A. Create GitHub OAuth App

1. Go to [GitHub Settings > Developer settings](https://github.com/settings/developers)
2. Click "OAuth Apps" in left sidebar
3. Click "New OAuth App"
4. Fill in:
   - **Application name**: `Physical AI Robotics Course` (or your name)
   - **Homepage URL**: `http://localhost:5000`
   - **Authorization callback URL**: `http://localhost:5000/api/auth/github/callback`
5. Click "Register application"
6. **Copy the Client ID**
7. Click "Generate a new client secret"
8. **Copy the Client Secret**

#### B. Add to .env

In your `.env` file:
```bash
GITHUB_CLIENT_ID=your_client_id_here
GITHUB_CLIENT_SECRET=your_client_secret_here
```

### Step 4: Frontend Setup

The frontend has:
- **useAuth hook** (`src/hooks/useAuth.ts`) - Authentication logic
- **AuthForm component** (`src/components/AuthForm.tsx`) - Login/signup form
- **NavbarLogin component** (`src/components/NavbarLogin.tsx`) - Navbar button

No additional config needed - they auto-connect to the backend!

---

## How It Works

### Authentication Flow

```
User Browser
    ↓
[Visits /auth page]
    ↓
[AuthForm component renders]
    ↓
[User enters email/password OR clicks Google/GitHub]
    ↓
[Frontend sends request to http://localhost:5000/api/auth/...]
    ↓
[Better Auth server handles authentication]
    ↓
[Session cookie set in browser]
    ↓
[User redirected to homepage]
    ↓
[NavbarLogin shows logged-in status]
```

### API Endpoints Available

```
POST   /api/auth/sign-up              - Register new user
POST   /api/auth/sign-in              - Login user
POST   /api/auth/sign-out             - Logout user
GET    /api/auth/session              - Get current session
GET    /api/auth/google               - Google OAuth redirect
GET    /api/auth/github               - GitHub OAuth redirect
POST   /api/auth/google/callback      - Google OAuth callback
POST   /api/auth/github/callback      - GitHub OAuth callback
```

---

## Component Documentation

### useAuth Hook

```typescript
import { useAuth } from '../hooks/useAuth';

function MyComponent() {
  const {
    user,                  // Current logged-in user
    isAuthenticated,       // Boolean
    isLoading,            // Loading state
    error,                // Error message if any
    signUp,               // (email, password, name) => Promise
    signIn,               // (email, password) => Promise
    signOut,              // () => Promise
    signInWithGoogle,     // () => void (redirects)
    signInWithGitHub,     // () => void (redirects)
  } = useAuth();

  return (
    // Your component...
  );
}
```

### AuthForm Component

Complete login/signup form with:
- Email/password sign up
- Email/password sign in
- Google OAuth button
- GitHub OAuth button
- Error handling
- Loading states

Usage:
```typescript
import { AuthForm } from '../components/AuthForm';

export default function AuthPage() {
  return <AuthForm />;
}
```

### NavbarLogin Component

Shows either:
- "Sign In" link (when not logged in)
- User profile button with dropdown (when logged in)
  - Shows user picture, name, email
  - Sign Out button

Usage:
```typescript
import { NavbarLogin } from '../components/NavbarLogin';

// In your navbar:
<NavbarLogin />
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Cannot find module 'better-auth'" | Run `npm install` |
| "connect ECONNREFUSED" on auth | Backend not running. Run `npm run server` |
| Google button doesn't work | Check GOOGLE_CLIENT_ID in .env |
| GitHub button doesn't work | Check GITHUB_CLIENT_ID in .env |
| Database locked error | Delete `data/auth.db` and restart |
| Session not persisting | Make sure cookies are enabled in browser |

### Common Errors

**"ENOENT: no such file or directory, open 'data/auth.db'"**
- The `data/` directory will be auto-created. If issue persists, manually create it:
```bash
mkdir data
```

**"Google OAuth error: redirect_uri_mismatch"**
- Ensure redirect URI in Google Console exactly matches:
  `http://localhost:5000/api/auth/google/callback`

**"GitHub OAuth error: bad_redirect_uri"**
- Ensure callback URL in GitHub App exactly matches:
  `http://localhost:5000/api/auth/github/callback`

---

## Production Deployment

### Environment Changes

Update `.env` for production:

```bash
# Production URLs
FRONTEND_URL=https://yourdomain.com
BETTER_AUTH_URL=https://yourdomain.com:5000

# Generate a strong secret
BETTER_AUTH_SECRET=<use openssl rand -hex 32>

# Update OAuth credentials
GOOGLE_CLIENT_ID=your_prod_client_id
GOOGLE_CLIENT_SECRET=your_prod_client_secret
GITHUB_CLIENT_ID=your_prod_client_id
GITHUB_CLIENT_SECRET=your_prod_client_secret
```

### OAuth Provider Settings

**Google Console:**
- Add production domain to "Authorized JavaScript origins"
- Update callback URL: `https://yourdomain.com:5000/api/auth/google/callback`

**GitHub:**
- Update Authorization callback URL: `https://yourdomain.com:5000/api/auth/github/callback`
- Update Homepage URL: `https://yourdomain.com`

### Server Deployment

Run on production:
```bash
npm install
npm run server  # Auth server
npm run build   # Build frontend
npm run serve   # Serve built frontend
```

Or use PM2 for process management:
```bash
npm install -g pm2
pm2 start auth-server.ts --name "auth-server"
pm2 start "npm run serve" --name "frontend"
```

---

## Features

✅ Email/password authentication
✅ Google OAuth login
✅ GitHub OAuth login
✅ User profile management
✅ Session persistence
✅ Automatic redirects
✅ Error handling
✅ Responsive design
✅ Dark mode support
✅ SQLite database

---

## Security Considerations

- 🔒 Passwords hashed securely
- 🔒 Session tokens in secure cookies
- 🔒 OAuth secrets never exposed to frontend
- 🔒 CORS properly configured
- 🔒 Environment variables for sensitive data
- 🔒 HTTPS-ready for production

---

## File Structure

```
project-root/
├── auth-server.ts                    # Better Auth server
├── .env.example                      # Environment template
├── .env                             # Your credentials (NOT in git)
├── data/
│   └── auth.db                      # SQLite database (auto-created)
├── src/
│   ├── hooks/
│   │   └── useAuth.ts               # Auth hook
│   ├── components/
│   │   ├── AuthForm.tsx             # Login/signup form
│   │   ├── AuthForm.module.css
│   │   ├── NavbarLogin.tsx          # Navbar button
│   │   └── NavbarLogin.module.css
│   ├── pages/
│   │   └── auth.tsx                 # Auth page
│   └── ...
├── package.json                      # npm dependencies
└── ...
```

---

## Next Steps

1. ✅ Complete setup and test locally
2. ⚠️ Change `BETTER_AUTH_SECRET` to a secure random value
3. 📧 Add email verification
4. 👤 Create user profile page
5. 📊 Add progress tracking to database
6. 🚀 Deploy to production

---

## Support & Resources

- [Better Auth Documentation](https://better-auth.vercel.app/)
- [Express.js Guide](https://expressjs.com/)
- [SQLite Docs](https://www.sqlite.org/)
- [Google OAuth Docs](https://developers.google.com/identity/protocols/oauth2)
- [GitHub OAuth Docs](https://docs.github.com/en/developers/apps/building-oauth-apps)

---

**Status**: ✅ Ready to use - Start with `npm install && npm run server` in one terminal and `npm run start` in another!
