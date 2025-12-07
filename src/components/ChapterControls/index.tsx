/**
 * Chapter Controls Component
 * Provides personalization and translation buttons at the start of each chapter.
 * Only visible to logged-in users.
 */

import React, { useState, useCallback } from 'react';
import { useAuth } from '../../context/AuthContext';
import { useChatContext } from '../../context/ChatContext';
import styles from './ChapterControls.module.css';

interface ChapterControlsProps {
  chapterTitle?: string;
  chapterContent?: string;
}

export function ChapterControls({ chapterTitle, chapterContent }: ChapterControlsProps) {
  const { user, isAuthenticated } = useAuth();
  const { apiUrl } = useChatContext();

  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showPersonalized, setShowPersonalized] = useState(false);
  const [showTranslated, setShowTranslated] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = useCallback(async () => {
    if (!user || !chapterContent) return;

    setIsPersonalizing(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth-token');
      const response = await fetch(`${apiUrl}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          content: chapterContent,
          chapter_title: chapterTitle,
          user_software_background: user.softwareBackground,
          user_hardware_background: user.hardwareBackground,
          experience_level: user.experienceLevel,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setPersonalizedContent(data.personalized_content);
        setShowPersonalized(true);
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to personalize content');
      }
    } catch (err) {
      setError('Network error. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  }, [apiUrl, user, chapterContent, chapterTitle]);

  const handleTranslate = useCallback(async () => {
    if (!chapterContent) return;

    setIsTranslating(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth-token');
      const headers: Record<string, string> = {
        'Content-Type': 'application/json',
      };
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }

      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          content: chapterContent,
          target_language: 'ur',
          chapter_title: chapterTitle,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setTranslatedContent(data.translated_content);
        setShowTranslated(true);
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to translate content');
      }
    } catch (err) {
      setError('Network error. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  }, [apiUrl, chapterContent, chapterTitle]);

  const togglePersonalized = () => {
    if (personalizedContent) {
      setShowPersonalized(!showPersonalized);
    } else {
      handlePersonalize();
    }
  };

  const toggleTranslated = () => {
    if (translatedContent) {
      setShowTranslated(!showTranslated);
    } else {
      handleTranslate();
    }
  };

  return (
    <div className={styles.container}>
      <div className={styles.controlsHeader}>
        <span className={styles.label}>Chapter Controls</span>
        <div className={styles.buttons}>
          {/* Personalize Button - Only for logged-in users */}
          {isAuthenticated && (
            <button
              className={`${styles.controlButton} ${styles.personalizeButton} ${showPersonalized ? styles.active : ''}`}
              onClick={togglePersonalized}
              disabled={isPersonalizing}
              title="Personalize content based on your background"
            >
              {isPersonalizing ? (
                <span className={styles.spinner} />
              ) : (
                <svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18">
                  <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
                </svg>
              )}
              <span>{showPersonalized ? 'Show Original' : 'Personalize'}</span>
            </button>
          )}

          {/* Translate to Urdu Button - Public, no login required */}
          <button
            className={`${styles.controlButton} ${styles.translateButton} ${showTranslated ? styles.active : ''}`}
            onClick={toggleTranslated}
            disabled={isTranslating}
            title="Translate content to Urdu"
          >
            {isTranslating ? (
              <span className={styles.spinner} />
            ) : (
              <svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18">
                <path d="M12.87 15.07l-2.54-2.51.03-.03c1.74-1.94 2.98-4.17 3.71-6.53H17V4h-7V2H8v2H1v2h11.17C11.5 7.92 10.44 9.75 9 11.35 8.07 10.32 7.3 9.19 6.69 8h-2c.73 1.63 1.73 3.17 2.98 4.56l-5.09 5.02L4 19l5-5 3.11 3.11.76-2.04zM18.5 10h-2L12 22h2l1.12-3h4.75L21 22h2l-4.5-12zm-2.62 7l1.62-4.33L19.12 17h-3.24z" />
              </svg>
            )}
            <span>{showTranslated ? 'Show English' : 'اردو میں پڑھیں'}</span>
          </button>
        </div>
      </div>

      {error && (
        <div className={styles.error}>
          <span>⚠️ {error}</span>
          <button onClick={() => setError(null)}>✕</button>
        </div>
      )}

      {/* Personalized Content Display */}
      {showPersonalized && personalizedContent && (
        <div className={styles.contentBox}>
          <div className={styles.contentHeader}>
            <span className={styles.contentLabel}>
              <svg viewBox="0 0 24 24" fill="currentColor" width="16" height="16">
                <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
              </svg>
              Personalized for your {user?.softwareBackground} software & {user?.hardwareBackground} hardware background
            </span>
          </div>
          <div
            className={styles.contentText}
            dangerouslySetInnerHTML={{ __html: personalizedContent }}
          />
        </div>
      )}

      {/* Translated Content Display */}
      {showTranslated && translatedContent && (
        <div className={`${styles.contentBox} ${styles.urduContent}`}>
          <div className={styles.contentHeader}>
            <span className={styles.contentLabel}>
              <svg viewBox="0 0 24 24" fill="currentColor" width="16" height="16">
                <path d="M12.87 15.07l-2.54-2.51.03-.03c1.74-1.94 2.98-4.17 3.71-6.53H17V4h-7V2H8v2H1v2h11.17C11.5 7.92 10.44 9.75 9 11.35 8.07 10.32 7.3 9.19 6.69 8h-2c.73 1.63 1.73 3.17 2.98 4.56l-5.09 5.02L4 19l5-5 3.11 3.11.76-2.04zM18.5 10h-2L12 22h2l1.12-3h4.75L21 22h2l-4.5-12zm-2.62 7l1.62-4.33L19.12 17h-3.24z" />
              </svg>
              اردو ترجمہ
            </span>
          </div>
          <div
            className={styles.contentText}
            dir="rtl"
            lang="ur"
            dangerouslySetInnerHTML={{ __html: translatedContent }}
          />
        </div>
      )}
    </div>
  );
}

export default ChapterControls;
