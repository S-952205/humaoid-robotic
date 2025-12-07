import express from 'express';
import axios from 'axios';
import cors from 'cors';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = process.env.PORT || 5000;

app.use(cors({
  origin: [
    'http://localhost:3000',
    'http://127.0.0.1:3000',
  ],
  credentials: true,
}));

app.use(express.json());

// Google OAuth Token Exchange
app.post('/api/auth/google/callback', async (req, res) => {
  try {
    const { code } = req.body;

    if (!code) {
      return res.status(400).json({ error: 'Authorization code is required' });
    }

    const googleClientId = process.env.GOOGLE_CLIENT_ID;
    const googleClientSecret = process.env.GOOGLE_CLIENT_SECRET;

    if (!googleClientId || !googleClientSecret) {
      return res.status(500).json({ error: 'Google OAuth credentials not configured' });
    }

    // Exchange authorization code for access token
    const tokenResponse = await axios.post('https://oauth2.googleapis.com/token', {
      code,
      client_id: googleClientId,
      client_secret: googleClientSecret,
      redirect_uri: `${process.env.FRONTEND_URL}/auth/callback`,
      grant_type: 'authorization_code',
    });

    const { access_token } = tokenResponse.data;

    // Fetch user profile using access token
    const userResponse = await axios.get('https://openidconnect.googleapis.com/v1/userinfo', {
      headers: { Authorization: `Bearer ${access_token}` },
    });

    const { sub, email, name, picture } = userResponse.data;

    res.json({
      user: {
        id: `google_${sub}`,
        email,
        name,
        image: picture,
      },
    });
  } catch (error) {
    console.error('Google OAuth error:', error);
    res.status(500).json({ error: 'Failed to authenticate with Google' });
  }
});

// GitHub OAuth Token Exchange
app.post('/api/auth/github/callback', async (req, res) => {
  try {
    const { code } = req.body;

    if (!code) {
      return res.status(400).json({ error: 'Authorization code is required' });
    }

    const githubClientId = process.env.GITHUB_CLIENT_ID;
    const githubClientSecret = process.env.GITHUB_CLIENT_SECRET;

    if (!githubClientId || !githubClientSecret) {
      return res.status(500).json({ error: 'GitHub OAuth credentials not configured' });
    }

    // Exchange authorization code for access token
    const tokenResponse = await axios.post(
      'https://github.com/login/oauth/access_token',
      {
        client_id: githubClientId,
        client_secret: githubClientSecret,
        code,
      },
      { headers: { Accept: 'application/json' } }
    );

    const { access_token } = tokenResponse.data;

    // Fetch user profile using access token
    const userResponse = await axios.get('https://api.github.com/user', {
      headers: { Authorization: `Bearer ${access_token}` },
    });

    const { id, login, name: displayName, avatar_url } = userResponse.data;

    // GitHub doesn't always provide email in the user endpoint
    let email = null;
    if (userResponse.data.email) {
      email = userResponse.data.email;
    } else {
      try {
        const emailResponse = await axios.get('https://api.github.com/user/emails', {
          headers: { Authorization: `Bearer ${access_token}` },
        });
        const primaryEmail = emailResponse.data.find((e: any) => e.primary);
        email = primaryEmail?.email || `${login}@github.local`;
      } catch {
        email = `${login}@github.local`;
      }
    }

    res.json({
      user: {
        id: `github_${id}`,
        email,
        name: displayName || login,
        image: avatar_url,
      },
    });
  } catch (error) {
    console.error('GitHub OAuth error:', error);
    res.status(500).json({ error: 'Failed to authenticate with GitHub' });
  }
});

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok' });
});

app.listen(PORT, () => {
  console.log(`OAuth server listening on http://localhost:${PORT}`);
  console.log(`Make sure to set environment variables:`);
  console.log(`- GOOGLE_CLIENT_ID`);
  console.log(`- GOOGLE_CLIENT_SECRET`);
  console.log(`- GITHUB_CLIENT_ID`);
  console.log(`- GITHUB_CLIENT_SECRET`);
  console.log(`- FRONTEND_URL (default: http://localhost:3000)`);
});

export default app;
