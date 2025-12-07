import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './auth.module.css';

type AuthMode = 'signin' | 'signup';

export default function AuthPage(): JSX.Element {
  const [mode, setMode] = useState<AuthMode>('signin');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [errors, setErrors] = useState<Record<string, string>>({});
  const { signIn, signUp, signInWithOAuth, isLoading, error, user } = useAuth();

  const handleOAuthLogin = async (provider: 'google' | 'github') => {
    try {
      await signInWithOAuth(provider);
    } catch (err) {
      console.error(`${provider} OAuth failed:`, err);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setErrors({});

    // Validation
    const newErrors: Record<string, string> = {};
    if (!email) newErrors.email = 'Email is required';
    if (!password) newErrors.password = 'Password is required';
    if (mode === 'signup' && !name) newErrors.name = 'Name is required';

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    try {
      if (mode === 'signin') {
        await signIn(email, password);
      } else {
        await signUp(email, password, name);
      }
      // Clear form on success
      setEmail('');
      setPassword('');
      setName('');
      // Redirect after login
      setTimeout(() => {
        window.location.href = '/humanoid-robot-book/';
      }, 1000);
    } catch (err) {
      // Error is handled by AuthContext
    }
  };

  // Show message if already logged in
  if (user) {
    return (
      <main className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Welcome, {user.name}!</h1>
          <p className={styles.subtitle}>You are already logged in</p>
          <div style={{ textAlign: 'center', marginTop: '2rem' }}>
            <img src={user.image} alt={user.name} style={{ borderRadius: '50%', width: '100px', height: '100px', marginBottom: '1rem' }} />
            <p>Email: {user.email}</p>
            <a href="/humanoid-robot-book/" style={{ color: '#2563eb', textDecoration: 'none', fontWeight: '600' }}>
              ← Back to Course
            </a>
          </div>
        </div>
      </main>
    );
  }

  return (
    <main className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>
            {mode === 'signin' ? 'Sign In' : 'Create Account'}
          </h1>
          <p className={styles.subtitle}>
            {mode === 'signin'
              ? 'Access your learning dashboard and progress'
              : 'Join thousands learning Physical AI & Robotics'}
          </p>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSubmit} className={styles.form}>
            {mode === 'signup' && (
              <div className={styles.formGroup}>
                <label htmlFor="name">Full Name</label>
                <input
                  id="name"
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  placeholder="Your full name"
                  className={errors.name ? styles.inputError : ''}
                  disabled={isLoading}
                />
                {errors.name && <span className={styles.errorText}>{errors.name}</span>}
              </div>
            )}

            <div className={styles.formGroup}>
              <label htmlFor="email">Email Address</label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="your@email.com"
                className={errors.email ? styles.inputError : ''}
                disabled={isLoading}
              />
              {errors.email && <span className={styles.errorText}>{errors.email}</span>}
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter your password"
                className={errors.password ? styles.inputError : ''}
                disabled={isLoading}
              />
              {errors.password && <span className={styles.errorText}>{errors.password}</span>}
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
                  setErrors({});
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
              className={styles.socialButton}
              disabled={isLoading}
              onClick={() => handleOAuthLogin('google')}
            >
              <span>🔍 Google</span>
            </button>
            <button
              className={styles.socialButton}
              disabled={isLoading}
              onClick={() => handleOAuthLogin('github')}
            >
              <span>🐙 GitHub</span>
            </button>
          </div>

          <p className={styles.privacy}>
            By continuing, you agree to our Terms of Service and Privacy Policy
          </p>
        </div>

        <div className={styles.infoSection}>
          <div className={styles.infoCard}>
            <h3>Learn at Your Pace</h3>
            <p>Access course materials 24/7 and learn whenever you want</p>
          </div>
          <div className={styles.infoCard}>
            <h3>Track Progress</h3>
            <p>Monitor your learning journey with detailed progress tracking</p>
          </div>
          <div className={styles.infoCard}>
            <h3>Community Support</h3>
            <p>Connect with thousands of robotics enthusiasts worldwide</p>
          </div>
        </div>
    </main>
  );
}
