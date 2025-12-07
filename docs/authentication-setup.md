---
sidebar_position: 100
---

# Authentication Setup Guide

This guide explains how to set up sign-in/sign-up functionality using BetterAuth in the Humanoid Robotics Course Book.

## Overview

BetterAuth is a modern authentication library that provides:
- Email/password authentication
- Social login (Google, GitHub)
- Session management
- Password reset functionality
- User profile management

## Quick Start

### 1. Environment Variables

Create a `.env.local` file in your project root:

```env
# Database Configuration
DATABASE_URL=file:./auth.db

# BetterAuth Secret (generate a secure random string)
AUTH_SECRET=your-super-secret-key-change-this

# Google OAuth (optional)
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret

# GitHub OAuth (optional)
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

### 2. Install Dependencies

```bash
npm install better-auth
```

### 3. Database Setup

BetterAuth supports multiple database options:

**SQLite (Development)**
```env
DATABASE_URL=file:./auth.db
```

**PostgreSQL (Production)**
```env
DATABASE_URL=postgresql://user:password@localhost:5432/robotics_auth
```

**MySQL**
```env
DATABASE_URL=mysql://user:password@localhost:3306/robotics_auth
```

## Server Setup

### Option 1: Node.js/Express (Recommended)

Create `server.js`:

```javascript
import express from 'express';
import { betterAuth } from 'better-auth';
import { database } from 'better-auth/database';

const app = express();
app.use(express.json());

// Initialize auth
const auth = betterAuth({
  database: database({
    type: 'sqlite',
    url: process.env.DATABASE_URL,
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: { enabled: true },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
});

// Mount auth routes
app.use('/api/auth', auth.handler);

// Start server
app.listen(3000, () => {
  console.log('Auth server running on http://localhost:3000');
});
```

Run with:
```bash
node server.js
```

### Option 2: Vercel (Recommended for Docusaurus)

Create `api/auth/[...auth].ts` in your Vercel project:

```typescript
import { betterAuth } from 'better-auth';
import { database } from 'better-auth/database';
import { NextRequest, NextResponse } from 'next/server';

const auth = betterAuth({
  database: database({
    type: 'postgres',
    url: process.env.DATABASE_URL,
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: { enabled: true },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
});

export async function handler(request: NextRequest) {
  return auth.handler(request);
}
```

### Option 3: Netlify Functions

Create `netlify/functions/auth/[...auth].ts`:

```typescript
import { betterAuth } from 'better-auth';
import { database } from 'better-auth/database';

const auth = betterAuth({
  database: database({
    type: 'postgres',
    url: process.env.DATABASE_URL,
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: { enabled: true },
});

export async function handler(event) {
  return auth.handler(event);
}
```

## Using Authentication in Components

### Example: Protected Page

```typescript
import { useAuth } from '@site/src/contexts/AuthContext';

export default function ProtectedPage() {
  const { user, isAuthenticated, signOut } = useAuth();

  if (!isAuthenticated) {
    return <p>Please sign in to access this content.</p>;
  }

  return (
    <div>
      <h1>Welcome, {user?.name}</h1>
      <p>Email: {user?.email}</p>
      <button onClick={signOut}>Sign Out</button>
    </div>
  );
}
```

### Accessing Auth Page

The authentication page is available at:
```
http://localhost:3000/auth
```

## OAuth Setup

### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project
3. Enable Google+ API
4. Create OAuth 2.0 credentials (Web application)
5. Add authorized redirect URI:
   ```
   http://localhost:3000/api/auth/google/callback
   ```
6. Copy Client ID and Client Secret to `.env.local`

### GitHub OAuth

1. Go to [GitHub Settings > Developer settings > OAuth Apps](https://github.com/settings/developers)
2. Create a new OAuth application
3. Set Authorization callback URL:
   ```
   http://localhost:3000/api/auth/github/callback
   ```
4. Copy Client ID and Client Secret to `.env.local`

## Available API Endpoints

### Authentication

- `POST /api/auth/signup` - Create new account
- `POST /api/auth/signin` - Sign in with email/password
- `POST /api/auth/signout` - Sign out
- `GET /api/auth/session` - Get current session

### Password Management

- `POST /api/auth/change-password` - Change user password
- `POST /api/auth/forget-password` - Request password reset
- `POST /api/auth/reset-password` - Reset password with token

### Social Login

- `GET /api/auth/google/authorize` - Start Google OAuth flow
- `GET /api/auth/github/authorize` - Start GitHub OAuth flow

## Database Schema

BetterAuth automatically creates the following tables:

- `users` - User accounts
- `sessions` - Active sessions
- `accounts` - OAuth account links
- `verifications` - Email verification codes

Example user structure:
```json
{
  "id": "user_123",
  "email": "student@example.com",
  "name": "John Doe",
  "emailVerified": true,
  "image": "https://example.com/avatar.jpg",
  "createdAt": "2024-12-07T12:00:00Z",
  "updatedAt": "2024-12-07T12:00:00Z"
}
```

## Security Best Practices

1. **Change AUTH_SECRET**: Use a strong, random string in production
2. **HTTPS Only**: Always use HTTPS in production
3. **Environment Variables**: Never commit `.env.local` to git
4. **Password Requirements**: Enforce strong passwords
5. **Rate Limiting**: Implement rate limiting on auth endpoints
6. **CSRF Protection**: Enable CSRF tokens
7. **Email Verification**: Require email verification for signups

## Troubleshooting

### "Cannot find module 'better-auth'"

Run:
```bash
npm install better-auth
```

### Database connection error

Check your `DATABASE_URL` environment variable and ensure the database is accessible.

### OAuth redirect mismatch

Ensure your OAuth application settings match the redirect URI in your code.

### CORS errors

If auth requests fail due to CORS, configure CORS in your server:

```javascript
import cors from 'cors';

app.use(cors({
  origin: process.env.FRONTEND_URL,
  credentials: true,
}));
```

## Next Steps

1. Deploy your auth server to production
2. Update `.env.local` with production URLs
3. Test signup and signin flow
4. Add user profile management
5. Create password reset email templates

## Resources

- [BetterAuth Documentation](https://better-auth.com)
- [Database Setup Guide](https://better-auth.com/docs/database)
- [OAuth Providers](https://better-auth.com/docs/oauth)
- [API Reference](https://better-auth.com/docs/api)

---

Need help? Check the course GitHub repository for more examples.
