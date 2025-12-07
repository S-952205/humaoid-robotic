import React, { useEffect, useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

export default function OAuthCallback(): JSX.Element {
  const { signOut } = useAuth();
  const [message, setMessage] = useState('Processing login...');
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const handleOAuthCallback = async () => {
      try {
        const params = new URLSearchParams(window.location.search);
        const code = params.get('code');
        const state = params.get('state');
        const storedState = sessionStorage.getItem('oauth_state');
        const provider = sessionStorage.getItem('oauth_provider');

        // Verify state for CSRF protection
        if (state !== storedState) {
          throw new Error('State mismatch - possible CSRF attack');
        }

        if (!code) {
          throw new Error('No authorization code received');
        }

        if (!provider) {
          throw new Error('No OAuth provider specified');
        }

        // Try to exchange code with backend server
        const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:5000';
        const endpoint = provider === 'google'
          ? `${backendUrl}/api/auth/google/callback`
          : `${backendUrl}/api/auth/github/callback`;

        const response = await fetch(endpoint, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ code }),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.error || `OAuth callback failed with status ${response.status}`);
        }

        const data = await response.json();
        const user = data.user;

        // Store user in localStorage
        localStorage.setItem('user', JSON.stringify(user));

        setMessage('Login successful! Redirecting...');

        // Redirect to home after a brief delay
        setTimeout(() => {
          window.location.href = '/humanoid-robot-book/';
        }, 1500);
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';
        setError(errorMessage);
        setMessage('Login failed');

        // If backend is not running, offer fallback
        if (errorMessage.includes('Failed to fetch') || errorMessage.includes('backend')) {
          setError(`${errorMessage}. Backend server (localhost:5000) may not be running. See .env.local.example for setup instructions.`);
        }
      } finally {
        // Clean up session storage
        sessionStorage.removeItem('oauth_state');
        sessionStorage.removeItem('oauth_provider');
      }
    };

    handleOAuthCallback();
  }, [signOut]);

  return (
    <main style={{
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center',
      minHeight: '100vh',
      flexDirection: 'column',
      gap: '1rem'
    }}>
      <div style={{ textAlign: 'center' }}>
        <h1>{message}</h1>
        {error && (
          <div style={{ color: '#dc2626', fontSize: '1.1rem', marginTop: '1rem' }}>
            <p>Error: {error}</p>
            <a href="/humanoid-robot-book/auth" style={{ color: '#2563eb', textDecoration: 'none' }}>
              ← Back to Login
            </a>
          </div>
        )}
      </div>
    </main>
  );
}
