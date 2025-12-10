import React, { useState } from 'react';
import styles from './styles.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  content: string;
  onPersonalized?: (personalizedContent: string) => void;
  apiUrl?: string;
}

export default function PersonalizeButton({
  chapterId,
  content,
  onPersonalized,
  apiUrl = '/api/personalize'
}: PersonalizeButtonProps): JSX.Element {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [error, setError] = useState('');

  const handlePersonalize = async () => {
    const token = localStorage.getItem('auth_token');
    if (!token) {
      setError('Please sign in to personalize content');
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      // First get user profile
      const profileResponse = await fetch('/api/auth/profile', {
        headers: { 'Authorization': `Bearer ${token}` },
      });

      if (!profileResponse.ok) {
        throw new Error('Please sign in to personalize content');
      }

      const profile = await profileResponse.json();

      // Then personalize content
      const response = await fetch(`${apiUrl}/content`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          content,
          chapter_id: chapterId,
          user_profile: {
            experience_level: profile.experience_level || 'intermediate',
            interests: profile.interests || [],
          },
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Personalization failed');
      }

      const data = await response.json();
      setIsPersonalized(true);
      onPersonalized?.(data.personalized_content);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Personalization failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setIsPersonalized(false);
    onPersonalized?.(content);
  };

  return (
    <div className={styles.container}>
      {error && <div className={styles.error}>{error}</div>}

      {isPersonalized ? (
        <button
          onClick={handleReset}
          className={`${styles.button} ${styles.reset}`}
          aria-label="Show original content"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8" />
            <path d="M3 3v5h5" />
          </svg>
          Show Original
        </button>
      ) : (
        <button
          onClick={handlePersonalize}
          disabled={isLoading}
          className={styles.button}
          aria-label="Personalize content for your experience level"
        >
          {isLoading ? (
            <>
              <span className={styles.spinner}></span>
              Personalizing...
            </>
          ) : (
            <>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
                <circle cx="12" cy="7" r="4" />
              </svg>
              Personalize for Me
            </>
          )}
        </button>
      )}

      {isPersonalized && (
        <span className={styles.badge}>
          Personalized
        </span>
      )}
    </div>
  );
}
