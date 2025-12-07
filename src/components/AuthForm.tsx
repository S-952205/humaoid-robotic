import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './AuthForm.module.css';

type AuthMode = 'signin' | 'signup';

export function AuthForm(): JSX.Element {
  const [mode, setMode] = useState<AuthMode>('signin');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [localError, setLocalError] = useState<string | null>(null);

  const { signUp, signIn, signInWithGoogle, signInWithGitHub, isLoading, user } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLocalError(null);

    try {
      if (mode === 'signin') {
        await signIn(email, password);
        // Redirect on success
        setTimeout(() => {
          window.location.href = '/humanoid-robot-book/';
        }, 500);
      } else {
        await signUp(email, password, name);
        // Redirect on success
        setTimeout(() => {
          window.location.href = '/humanoid-robot-book/';
        }, 500);
      }
    } catch (err) {
      setLocalError(err instanceof Error ? err.message : 'Authentication failed');
    }
  };

  if (user) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Welcome, {user.name}!</h1>
          <p className={styles.subtitle}>You are already logged in</p>
          <div style={{ textAlign: 'center', marginTop: '2rem' }}>
            {user.image && (
              <img
                src={user.image}
                alt={user.name}
                style={{
                  borderRadius: '50%',
                  width: '100px',
                  height: '100px',
                  marginBottom: '1rem',
                }}
              />
            )}
            <p>Email: {user.email}</p>
            <a
              href="/humanoid-robot-book/"
              style={{
                color: '#2563eb',
                textDecoration: 'none',
                fontWeight: '600',
              }}
            >
              ← Back to Course
            </a>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1>{mode === 'signin' ? 'Sign In' : 'Create Account'}</h1>
        <p className={styles.subtitle}>
          {mode === 'signin'
            ? 'Access your learning dashboard'
            : 'Join the course'}
        </p>

        {localError && <div className={styles.error}>{localError}</div>}

        <form onSubmit={handleSubmit} className={styles.form}>
          {mode === 'signup' && (
            <div className={styles.formGroup}>
              <label htmlFor="name">Full Name</label>
              <input
                id="name"
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="Your name"
                disabled={isLoading}
                required
              />
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="email">Email</label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="your@email.com"
              disabled={isLoading}
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Enter password"
              disabled={isLoading}
              required
            />
          </div>

          <button
            type="submit"
            disabled={isLoading}
            className={styles.submitButton}
          >
            {isLoading
              ? mode === 'signin'
                ? 'Signing in...'
                : 'Creating account...'
              : mode === 'signin'
              ? 'Sign In'
              : 'Create Account'}
          </button>
        </form>

        <div className={styles.toggle}>
          <p>
            {mode === 'signin' ? "Don't have an account?" : 'Already have an account?'}{' '}
            <button
              type="button"
              onClick={() => {
                setMode(mode === 'signin' ? 'signup' : 'signin');
                setLocalError(null);
              }}
              className={styles.toggleButton}
            >
              {mode === 'signin' ? 'Sign Up' : 'Sign In'}
            </button>
          </p>
        </div>

        <div className={styles.divider}>Or continue with</div>

        <div className={styles.socialButtons}>
          <button
            type="button"
            className={styles.socialButton}
            disabled={isLoading}
            onClick={signInWithGoogle}
          >
            <span>🔍 Google</span>
          </button>
          <button
            type="button"
            className={styles.socialButton}
            disabled={isLoading}
            onClick={signInWithGitHub}
          >
            <span>🐙 GitHub</span>
          </button>
        </div>
      </div>
    </div>
  );
}
