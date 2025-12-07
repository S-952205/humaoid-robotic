# Authentication Implementation Guide

## Overview

This project now includes complete user authentication with sign-in and sign-up functionality using **BetterAuth**. The authentication system is integrated into the Docusaurus site and provides:

- Email/password authentication
- Social login (Google, GitHub)
- Session management
- Password recovery
- User profile tracking

## What's New

### 1. **Authentication Page** (`/auth`)
- Clean, modern sign-in/sign-up interface
- Email and password validation
- Social login buttons (Google, GitHub)
- Mobile-responsive design
- Dark mode support

### 2. **Auth Context** (`src/contexts/AuthContext.tsx`)
- React Context for managing authentication state
- Hooks: `useAuth()` for accessing user state
- Methods: `signIn()`, `signUp()`, `signOut()`

### 3. **Auth API Server** (`auth-server.ts`)
- Example BetterAuth server setup
- Database integration (SQLite, PostgreSQL, MySQL)
- Social provider configuration

### 4. **Authentication Documentation** (`docs/authentication-setup.md`)
- Complete setup guide
- Database configuration
- OAuth provider setup
- API endpoints reference
- Security best practices
- Troubleshooting guide

### 5. **Auth Button Component** (`src/components/AuthButton.tsx`)
- Navbar authentication button
- User dropdown menu
- Responsive design

## Quick Start

### Install Dependencies
```bash
npm install better-auth
```

### Configure Environment
Copy `.env.example` to `.env.local` and fill in your values:

```bash
cp .env.example .env.local
```

### Set Up Database
For development, BetterAuth uses SQLite by default:
```env
DATABASE_URL=file:./auth.db
AUTH_SECRET=your-secret-key
```

### Start Authentication Server

Create a simple Node.js server (e.g., `server.js`):

```javascript
import express from 'express';
import { betterAuth } from 'better-auth';
import { database } from 'better-auth/database';

const app = express();
app.use(express.json());

const auth = betterAuth({
  database: database({
    type: 'sqlite',
    url: process.env.DATABASE_URL,
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: { enabled: true },
});

app.use('/api/auth', auth.handler);
app.listen(3000);
```

Run:
```bash
node server.js
```

## File Structure

```
humanoid-robot-book/
├── auth-server.ts                    # BetterAuth server setup
├── .env.example                      # Environment variables template
├── src/
│   ├── contexts/
│   │   └── AuthContext.tsx          # Auth state management
│   ├── components/
│   │   ├── AuthButton.tsx           # Navbar auth button
│   │   └── AuthButton.module.css    # Auth button styles
│   └── pages/
│       ├── auth.tsx                 # Sign-in/Sign-up page
│       └── auth.module.css          # Auth page styles
├── docs/
│   ├── intro.md                     # Updated with auth info
│   └── authentication-setup.md      # Complete setup guide
└── docusaurus.config.js             # Updated with blog config
```

## API Endpoints

Once the auth server is running, the following endpoints are available:

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

## Usage in Components

### Using Auth Hook

```typescript
import { useAuth } from '@site/src/contexts/AuthContext';

export default function MyComponent() {
  const { user, isAuthenticated, signOut } = useAuth();

  if (!isAuthenticated) {
    return <p>Please sign in</p>;
  }

  return (
    <div>
      <h1>Welcome, {user?.name}</h1>
      <button onClick={signOut}>Sign Out</button>
    </div>
  );
}
```

### Protecting Routes

```typescript
import { useAuth } from '@site/src/contexts/AuthContext';
import Link from '@docusaurus/Link';

export default function ProtectedComponent() {
  const { isAuthenticated } = useAuth();

  if (!isAuthenticated) {
    return (
      <div>
        <p>This content requires authentication.</p>
        <Link to="/auth">Sign in to continue</Link>
      </div>
    );
  }

  return <div>Protected content here</div>;
}
```

## OAuth Setup

### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project
3. Enable Google+ API
4. Create OAuth credentials:
   - Application type: Web application
   - Authorized redirect URIs:
     - `http://localhost:3000/api/auth/google/callback` (development)
     - `https://yourdomain.com/api/auth/google/callback` (production)
5. Copy Client ID and Secret to `.env.local`

### GitHub OAuth

1. Go to [GitHub Settings > Developer settings > OAuth Apps](https://github.com/settings/developers)
2. Create New OAuth App
3. Set Authorization callback URL:
   - `http://localhost:3000/api/auth/github/callback` (development)
   - `https://yourdomain.com/api/auth/github/callback` (production)
4. Copy Client ID and Secret to `.env.local`

## Database Configuration

### SQLite (Development)
```env
DATABASE_URL=file:./auth.db
```

### PostgreSQL (Production Recommended)
```env
DATABASE_URL=postgresql://user:password@localhost:5432/robotics
```

### MySQL
```env
DATABASE_URL=mysql://user:password@localhost:3306/robotics
```

## Security Considerations

1. **Change AUTH_SECRET**: Use a strong, random string
   ```bash
   # Linux/Mac
   openssl rand -hex 32
   ```

2. **Use HTTPS**: Always use HTTPS in production

3. **Environment Variables**: Never commit `.env.local` to git
   - Add to `.gitignore`:
   ```
   .env.local
   .env.*.local
   auth.db
   ```

4. **Password Requirements**:
   - Minimum 8 characters
   - Mix of uppercase, lowercase, numbers, and symbols
   - Consider requiring email verification

5. **Rate Limiting**: Implement rate limiting on auth endpoints

6. **CSRF Protection**: Enable CSRF tokens in production

## Deployment

### Vercel
```bash
vercel env add DATABASE_URL
vercel env add AUTH_SECRET
vercel env add GOOGLE_CLIENT_ID
vercel env add GOOGLE_CLIENT_SECRET
```

### Netlify
Add environment variables in Netlify Dashboard:
- Settings > Build & Deploy > Environment
- Add: `DATABASE_URL`, `AUTH_SECRET`, `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`

### Docker
Create `Dockerfile`:
```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY . .
CMD ["node", "server.js"]
```

## Troubleshooting

### "Cannot find module 'better-auth'"
```bash
npm install better-auth
```

### Database connection error
- Check `DATABASE_URL` is correct
- Ensure database is running
- Verify database user permissions

### OAuth errors
- Check Client ID and Secret in `.env.local`
- Verify redirect URIs match exactly
- Ensure OAuth app is enabled

### CORS issues
Add CORS middleware to your server:
```javascript
import cors from 'cors';
app.use(cors({ origin: process.env.FRONTEND_URL, credentials: true }));
```

## Next Steps

1. Deploy auth server to production
2. Set up OAuth providers
3. Configure production database
4. Add email verification
5. Implement user profiles
6. Add progress tracking
7. Create admin dashboard

## Resources

- [BetterAuth Documentation](https://better-auth.com)
- [Database Guide](https://better-auth.com/docs/database)
- [OAuth Documentation](https://better-auth.com/docs/oauth)
- [API Reference](https://better-auth.com/docs/api)

## Support

For issues or questions:
1. Check the [Authentication Setup Guide](docs/authentication-setup.md)
2. Review the [troubleshooting section](#troubleshooting)
3. Visit [GitHub Issues](https://github.com/Syed-Sufyan/humanoid-robot-book/issues)

---

**Last Updated**: December 7, 2024
