import React, { createContext, useContext, useState, useCallback, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  image?: string;
}

interface AuthContextType {
  user: User | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, name: string) => Promise<void>;
  signInWithOAuth: (provider: 'google' | 'github') => Promise<void>;
  signOut: () => void;
  error: string | null;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Load user from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem('user');
    if (stored) {
      try {
        setUser(JSON.parse(stored));
      } catch (e) {
        localStorage.removeItem('user');
      }
    }
  }, []);

  const signIn = useCallback(async (email: string, password: string) => {
    setIsLoading(true);
    setError(null);
    try {
      if (!email || !password) {
        throw new Error('Email and password required');
      }

      // Simple localStorage-based auth (no server needed)
      const newUser: User = {
        id: 'user_' + Date.now(),
        email,
        name: email.split('@')[0],
        image: `https://i.pravatar.cc/150?u=${email}`,
      };

      setUser(newUser);
      localStorage.setItem('user', JSON.stringify(newUser));
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to sign in';
      setError(message);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  const signUp = useCallback(async (email: string, password: string, name: string) => {
    setIsLoading(true);
    setError(null);
    try {
      if (!email || !password || !name) {
        throw new Error('All fields required');
      }

      // Simple localStorage-based auth
      const newUser: User = {
        id: 'user_' + Date.now(),
        email,
        name,
        image: `https://i.pravatar.cc/150?u=${email}`,
      };

      setUser(newUser);
      localStorage.setItem('user', JSON.stringify(newUser));
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to sign up';
      setError(message);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  const signInWithOAuth = useCallback(async (provider: 'google' | 'github') => {
    setIsLoading(true);
    setError(null);
    try {
      // Simulate OAuth login - in a real app, this would redirect to OAuth provider
      // For this demo, we'll create a mock user with OAuth provider info
      const providerName = provider === 'google' ? 'Google' : 'GitHub';
      const newUser: User = {
        id: `${provider}_${Date.now()}`,
        email: `user-${Date.now()}@${provider}.local`,
        name: `${providerName} User`,
        image: `https://i.pravatar.cc/150?u=${provider}_${Date.now()}@${provider}.local`,
      };

      setUser(newUser);
      localStorage.setItem('user', JSON.stringify(newUser));

      // Redirect after successful OAuth login
      setTimeout(() => {
        window.location.href = '/humanoid-robot-book/';
      }, 1000);
    } catch (err) {
      const message = err instanceof Error ? err.message : `Failed to sign in with ${provider}`;
      setError(message);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  const signOut = useCallback(() => {
    setUser(null);
    localStorage.removeItem('user');
    setError(null);
  }, []);

  return (
    <AuthContext.Provider
      value={{
        user,
        isLoading,
        isAuthenticated: !!user,
        signIn,
        signUp,
        signInWithOAuth,
        signOut,
        error,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    // Return default context if not in provider
    return {
      user: null,
      isLoading: false,
      isAuthenticated: false,
      signIn: async () => {},
      signUp: async () => {},
      signInWithOAuth: async () => {},
      signOut: () => {},
      error: null,
    };
  }
  return context;
};
