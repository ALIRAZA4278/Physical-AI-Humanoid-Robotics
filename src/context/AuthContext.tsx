/**
 * Authentication context for managing user state across the application.
 * Integrates with Better-Auth backend API.
 */

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useEffect,
  ReactNode,
} from 'react';

// User interface with background information for personalization
export interface User {
  id: string;
  email: string;
  name: string;
  softwareBackground: string;
  hardwareBackground: string;
  preferredLanguage: 'en' | 'ur';
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  createdAt: string;
}

interface AuthContextValue {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  signup: (data: SignupData) => Promise<{ success: boolean; error?: string }>;
  logout: () => Promise<void>;
  updatePreferences: (prefs: Partial<UserPreferences>) => Promise<void>;
}

interface SignupData {
  email: string;
  password: string;
  name: string;
  softwareBackground: string;
  hardwareBackground: string;
}

interface UserPreferences {
  preferredLanguage: 'en' | 'ur';
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
}

const AuthContext = createContext<AuthContextValue | null>(null);

interface AuthProviderProps {
  children: ReactNode;
  apiUrl: string;
}

export function AuthProvider({ children, apiUrl }: AuthProviderProps) {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check for existing session on mount
  useEffect(() => {
    const checkAuth = async () => {
      try {
        const token = localStorage.getItem('auth-token');
        if (token) {
          const response = await fetch(`${apiUrl}/api/auth/me`, {
            headers: {
              Authorization: `Bearer ${token}`,
            },
          });
          if (response.ok) {
            const userData = await response.json();
            setUser(userData);
          } else {
            localStorage.removeItem('auth-token');
          }
        }
      } catch (error) {
        console.error('Auth check failed:', error);
      } finally {
        setIsLoading(false);
      }
    };

    checkAuth();
  }, [apiUrl]);

  const login = useCallback(
    async (email: string, password: string) => {
      try {
        const response = await fetch(`${apiUrl}/api/auth/login`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ email, password }),
        });

        const data = await response.json();

        if (response.ok) {
          localStorage.setItem('auth-token', data.token);
          setUser(data.user);
          return { success: true };
        } else {
          return { success: false, error: data.detail || 'Login failed' };
        }
      } catch (error) {
        return { success: false, error: 'Network error. Please try again.' };
      }
    },
    [apiUrl]
  );

  const signup = useCallback(
    async (signupData: SignupData) => {
      try {
        const response = await fetch(`${apiUrl}/api/auth/signup`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            ...signupData,
            preferred_language: 'en',
            experience_level: 'beginner',
          }),
        });

        const data = await response.json();

        if (response.ok) {
          localStorage.setItem('auth-token', data.token);
          setUser(data.user);
          return { success: true };
        } else {
          return { success: false, error: data.detail || 'Signup failed' };
        }
      } catch (error) {
        return { success: false, error: 'Network error. Please try again.' };
      }
    },
    [apiUrl]
  );

  const logout = useCallback(async () => {
    try {
      const token = localStorage.getItem('auth-token');
      if (token) {
        await fetch(`${apiUrl}/api/auth/logout`, {
          method: 'POST',
          headers: {
            Authorization: `Bearer ${token}`,
          },
        });
      }
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      localStorage.removeItem('auth-token');
      setUser(null);
    }
  }, [apiUrl]);

  const updatePreferences = useCallback(
    async (prefs: Partial<UserPreferences>) => {
      try {
        const token = localStorage.getItem('auth-token');
        if (!token) return;

        const response = await fetch(`${apiUrl}/api/auth/preferences`, {
          method: 'PATCH',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${token}`,
          },
          body: JSON.stringify(prefs),
        });

        if (response.ok) {
          const updatedUser = await response.json();
          setUser(updatedUser);
        }
      } catch (error) {
        console.error('Failed to update preferences:', error);
      }
    },
    [apiUrl]
  );

  const value: AuthContextValue = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    signup,
    logout,
    updatePreferences,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
