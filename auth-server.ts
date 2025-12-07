import { betterAuth } from "better-auth";
import { sqlite } from "better-auth/database";
import express from "express";
import cors from "cors";
import path from "path";
import fs from "fs";
import dotenv from "dotenv";

dotenv.config();

const app = express();
const PORT = process.env.PORT || 5000;

// Create data directory for SQLite database
const dataDir = path.join(process.cwd(), "data");
if (!fs.existsSync(dataDir)) {
  fs.mkdirSync(dataDir, { recursive: true });
}

// Initialize better-auth with SQLite
const auth = betterAuth({
  appName: "Physical AI Robotics Course",
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:5000",
  basePath: "/api/auth",
  secret: process.env.BETTER_AUTH_SECRET || "dev-secret-key-change-in-production",

  database: sqlite({
    file: path.join(dataDir, "auth.db"),
  }),

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },

  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || "",
      clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
    },
  },

  user: {
    additionalFields: {
      role: {
        type: "string",
        defaultValue: "student",
      },
    },
  },
});

// CORS configuration
app.use(
  cors({
    origin: [
      "http://localhost:3000",
      "http://127.0.0.1:3000",
      process.env.FRONTEND_URL || "http://localhost:3000",
    ],
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Mount better-auth routes
app.all("/api/auth/*", (req, res) => {
  auth.handler(req, res);
});

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({ status: "ok", auth: "ready" });
});

app.listen(PORT, () => {
  console.log(`✅ Better Auth server running on http://localhost:${PORT}`);
  console.log(`📝 API Base: http://localhost:${PORT}/api/auth`);
  console.log(`\n📚 Available endpoints:`);
  console.log(`   POST   /api/auth/sign-up         - Register user`);
  console.log(`   POST   /api/auth/sign-in         - Login user`);
  console.log(`   POST   /api/auth/sign-out        - Logout user`);
  console.log(`   GET    /api/auth/session         - Get current session`);
  console.log(`   GET    /api/auth/google          - Google OAuth`);
  console.log(`   GET    /api/auth/github          - GitHub OAuth`);
  console.log(`\n⚠️  Environment variables needed:`);
  console.log(`   GOOGLE_CLIENT_ID`);
  console.log(`   GOOGLE_CLIENT_SECRET`);
  console.log(`   GITHUB_CLIENT_ID`);
  console.log(`   GITHUB_CLIENT_SECRET`);
});

export default auth;
