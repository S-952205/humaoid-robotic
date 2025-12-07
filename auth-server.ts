/**
 * Simple BetterAuth server setup for the Humanoid Robotics Book
 * This file demonstrates how to set up authentication with BetterAuth
 *
 * To use this in a full Node.js/Express setup:
 * 1. Install: npm install better-auth express dotenv
 * 2. Create a .env file with DATABASE_URL and AUTH_SECRET
 * 3. Use this code in your Express server
 *
 * For a Docusaurus-only setup, integrate with a serverless platform
 * like Vercel, AWS Lambda, or Netlify Functions
 */

import { betterAuth } from 'better-auth';
import { database } from 'better-auth/database';

// Database configuration - uses better-auth's built-in database plugin
// Supports: SQLite, PostgreSQL, MySQL, MongoDB
const db = database({
  type: 'sqlite', // Change to 'postgres' or 'mysql' for production
  url: process.env.DATABASE_URL || 'file:./auth.db',
});

// Initialize BetterAuth
const auth = betterAuth({
  database: db,
  secret: process.env.AUTH_SECRET || 'your-secret-key-change-in-production',

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true for production
  },

  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || '',
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || '',
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || '',
      clientSecret: process.env.GITHUB_CLIENT_SECRET || '',
    },
  },

  user: {
    additionalFields: {
      role: {
        type: 'string',
        defaultValue: 'student',
      },
    },
  },

  plugins: [],
});

/**
 * Example Express middleware/routes setup
 *
 * If using Express, add this to your server:
 *
 * import express from 'express';
 *
 * const app = express();
 * app.use(express.json());
 *
 * // Mount BetterAuth routes
 * app.use('/api/auth', auth.handler);
 *
 * // Your other routes...
 *
 * app.listen(3000, () => {
 * console.log('Server running on http://localhost:3000');
 * });
 */

/**
 * API Endpoints that will be available:
 *
 * POST /api/auth/signup
 *   Body: { email, password, name }
 *   Returns: { user, session }
 *
 * POST /api/auth/signin
 *   Body: { email, password }
 *   Returns: { user, session }
 *
 * POST /api/auth/signout
 *   Returns: { success: true }
 *
 * GET /api/auth/session
 *   Returns: { user, session } or null
 *
 * POST /api/auth/change-password
 *   Body: { currentPassword, newPassword }
 *   Returns: { success: true }
 *
 * POST /api/auth/forget-password
 *   Body: { email }
 *   Returns: { success: true }
 *
 * POST /api/auth/reset-password
 *   Body: { token, newPassword }
 *   Returns: { success: true }
 *
 * For OAuth providers:
 * GET /api/auth/google/authorize
 * GET /api/auth/google/callback
 * GET /api/auth/github/authorize
 * GET /api/auth/github/callback
 */

export default auth;
