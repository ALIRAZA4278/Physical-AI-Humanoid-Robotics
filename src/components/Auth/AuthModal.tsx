/**
 * Authentication modal component for login/signup.
 * Collects user background information during signup for personalization.
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './AuthModal.module.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialMode?: 'login' | 'signup';
}

export function AuthModal({ isOpen, onClose, initialMode = 'login' }: AuthModalProps) {
  const [mode, setMode] = useState<'login' | 'signup'>(initialMode);

  // Sync mode with initialMode when it changes (fixes same modal for both buttons)
  useEffect(() => {
    setMode(initialMode);
  }, [initialMode]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: '',
  });

  const { login, signup } = useAuth();

  if (!isOpen) return null;

  const handleInputChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>
  ) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    setError(null);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      if (mode === 'login') {
        const result = await login(formData.email, formData.password);
        if (result.success) {
          onClose();
        } else {
          setError(result.error || 'Login failed');
        }
      } else {
        if (!formData.softwareBackground || !formData.hardwareBackground) {
          setError('Please fill in your background information for personalized content');
          setIsLoading(false);
          return;
        }
        const result = await signup({
          email: formData.email,
          password: formData.password,
          name: formData.name,
          softwareBackground: formData.softwareBackground,
          hardwareBackground: formData.hardwareBackground,
        });
        if (result.success) {
          onClose();
        } else {
          setError(result.error || 'Signup failed');
        }
      }
    } finally {
      setIsLoading(false);
    }
  };

  const switchMode = () => {
    setMode((prev) => (prev === 'login' ? 'signup' : 'login'));
    setError(null);
  };

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose} aria-label="Close">
          <svg viewBox="0 0 24 24" fill="currentColor" width="20" height="20">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
          </svg>
        </button>

        <h2 className={styles.title}>
          {mode === 'login' ? 'Welcome Back' : 'Create Account'}
        </h2>
        <p className={styles.subtitle}>
          {mode === 'login'
            ? 'Sign in to access personalized content'
            : 'Tell us about yourself for personalized learning'}
        </p>

        <form onSubmit={handleSubmit} className={styles.form}>
          {mode === 'signup' && (
            <div className={styles.formGroup}>
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                name="name"
                value={formData.name}
                onChange={handleInputChange}
                placeholder="Enter your name"
                required
              />
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleInputChange}
              placeholder="your@email.com"
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleInputChange}
              placeholder={mode === 'login' ? 'Enter password' : 'Create password (min 8 chars)'}
              minLength={8}
              required
            />
          </div>

          {mode === 'signup' && (
            <>
              <div className={styles.divider}>
                <span>Background Questions</span>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="softwareBackground">
                  Software Development Experience
                  <span className={styles.hint}>
                    This helps us personalize content to your skill level
                  </span>
                </label>
                <select
                  id="softwareBackground"
                  name="softwareBackground"
                  value={formData.softwareBackground}
                  onChange={handleInputChange}
                  required
                >
                  <option value="">Select your experience level</option>
                  <option value="none">No programming experience</option>
                  <option value="beginner">Beginner (learning basics, simple scripts)</option>
                  <option value="intermediate">Intermediate (built projects, familiar with frameworks)</option>
                  <option value="advanced">Advanced (professional experience, complex systems)</option>
                  <option value="expert">Expert (10+ years, architecture & system design)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="hardwareBackground">
                  Hardware & Robotics Experience
                  <span className={styles.hint}>
                    Tell us about your robotics/embedded systems background
                  </span>
                </label>
                <select
                  id="hardwareBackground"
                  name="hardwareBackground"
                  value={formData.hardwareBackground}
                  onChange={handleInputChange}
                  required
                >
                  <option value="">Select your experience level</option>
                  <option value="none">No hardware/robotics experience</option>
                  <option value="hobbyist">Hobbyist (Arduino, basic electronics, DIY projects)</option>
                  <option value="student">Student (coursework in robotics/embedded systems)</option>
                  <option value="professional">Professional (work experience with robots/hardware)</option>
                  <option value="expert">Expert (designed robotic systems, research experience)</option>
                </select>
              </div>
            </>
          )}

          {error && <div className={styles.error}>{error}</div>}

          <button type="submit" className={styles.submitButton} disabled={isLoading}>
            {isLoading ? (
              <span className={styles.spinner} />
            ) : mode === 'login' ? (
              'Sign In'
            ) : (
              'Create Account'
            )}
          </button>
        </form>

        <div className={styles.switchMode}>
          {mode === 'login' ? (
            <>
              Don't have an account?{' '}
              <button type="button" onClick={switchMode}>
                Sign up
              </button>
            </>
          ) : (
            <>
              Already have an account?{' '}
              <button type="button" onClick={switchMode}>
                Sign in
              </button>
            </>
          )}
        </div>
      </div>
    </div>
  );
}
