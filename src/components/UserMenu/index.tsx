/**
 * User Menu Component
 * Shows login/signup button for guests, user profile menu for authenticated users.
 * Positioned in the top-right corner of the page.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import { AuthModal } from '../Auth/AuthModal';
import styles from './UserMenu.module.css';

export function UserMenu() {
  const { user, isAuthenticated, isLoading, logout, updatePreferences } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [authMode, setAuthMode] = useState<'login' | 'signup'>('login');
  const [showDropdown, setShowDropdown] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setShowDropdown(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleLogin = () => {
    setAuthMode('login');
    setShowAuthModal(true);
  };

  const handleSignup = () => {
    setAuthMode('signup');
    setShowAuthModal(true);
  };

  const handleLogout = async () => {
    await logout();
    setShowDropdown(false);
  };

  const handleLanguageChange = async (lang: 'en' | 'ur') => {
    await updatePreferences({ preferredLanguage: lang });
  };

  if (isLoading) {
    return null;
  }

  return (
    <>
      <div className={styles.container}>
        {isAuthenticated && user ? (
          <div className={styles.userSection} ref={dropdownRef}>
            <button
              className={styles.userButton}
              onClick={() => setShowDropdown(!showDropdown)}
              aria-expanded={showDropdown}
              aria-haspopup="true"
            >
              <span className={styles.avatar}>
                {user.name.charAt(0).toUpperCase()}
              </span>
              <span className={styles.userName}>{user.name}</span>
              <svg
                className={`${styles.chevron} ${showDropdown ? styles.chevronUp : ''}`}
                viewBox="0 0 24 24"
                fill="currentColor"
                width="16"
                height="16"
              >
                <path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z" />
              </svg>
            </button>

            {showDropdown && (
              <div className={styles.dropdown}>
                <div className={styles.dropdownHeader}>
                  <div className={styles.avatarLarge}>
                    {user.name.charAt(0).toUpperCase()}
                  </div>
                  <div className={styles.userInfo}>
                    <span className={styles.userNameLarge}>{user.name}</span>
                    <span className={styles.userEmail}>{user.email}</span>
                  </div>
                </div>

                <div className={styles.dropdownDivider} />

                <div className={styles.dropdownSection}>
                  <span className={styles.sectionLabel}>Your Background</span>
                  <div className={styles.backgroundInfo}>
                    <div className={styles.backgroundItem}>
                      <span className={styles.backgroundLabel}>Software:</span>
                      <span className={styles.backgroundValue}>{user.softwareBackground}</span>
                    </div>
                    <div className={styles.backgroundItem}>
                      <span className={styles.backgroundLabel}>Hardware:</span>
                      <span className={styles.backgroundValue}>{user.hardwareBackground}</span>
                    </div>
                  </div>
                </div>

                <div className={styles.dropdownDivider} />

                <div className={styles.dropdownSection}>
                  <span className={styles.sectionLabel}>Preferred Language</span>
                  <div className={styles.languageSelector}>
                    <button
                      className={`${styles.langButton} ${user.preferredLanguage === 'en' ? styles.langActive : ''}`}
                      onClick={() => handleLanguageChange('en')}
                    >
                      English
                    </button>
                    <button
                      className={`${styles.langButton} ${user.preferredLanguage === 'ur' ? styles.langActive : ''}`}
                      onClick={() => handleLanguageChange('ur')}
                    >
                      اردو
                    </button>
                  </div>
                </div>

                <div className={styles.dropdownDivider} />

                <button className={styles.logoutButton} onClick={handleLogout}>
                  <svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18">
                    <path d="M17 7l-1.41 1.41L18.17 11H8v2h10.17l-2.58 2.58L17 17l5-5zM4 5h8V3H4c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h8v-2H4V5z" />
                  </svg>
                  Sign Out
                </button>
              </div>
            )}
          </div>
        ) : (
          <div className={styles.authButtons}>
            <button className={styles.loginButton} onClick={handleLogin}>
              Sign In
            </button>
            <button className={styles.signupButton} onClick={handleSignup}>
              Sign Up
            </button>
          </div>
        )}
      </div>

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        initialMode={authMode}
      />
    </>
  );
}

export default UserMenu;
