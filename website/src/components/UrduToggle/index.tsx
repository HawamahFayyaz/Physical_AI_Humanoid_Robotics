import React, { useState } from 'react';
import styles from './styles.module.css';

interface UrduToggleProps {
  chapterId: string;
  content: string;
  onTranslated?: (translatedContent: string, isUrdu: boolean) => void;
  apiUrl?: string;
}

export default function UrduToggle({
  chapterId,
  content,
  onTranslated,
  apiUrl = '/api/translate'
}: UrduToggleProps): JSX.Element {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [cachedTranslation, setCachedTranslation] = useState<string | null>(null);

  const handleToggle = async () => {
    if (isUrdu) {
      // Switch back to English
      setIsUrdu(false);
      onTranslated?.(content, false);
      return;
    }

    // Use cached translation if available
    if (cachedTranslation) {
      setIsUrdu(true);
      onTranslated?.(cachedTranslation, true);
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      const response = await fetch(`${apiUrl}/urdu`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content,
          chapter_id: chapterId,
          preserve_code: true,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Translation failed');
      }

      const data = await response.json();
      setCachedTranslation(data.translated_content);
      setIsUrdu(true);
      onTranslated?.(data.translated_content, true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      {error && <div className={styles.error}>{error}</div>}

      <button
        onClick={handleToggle}
        disabled={isLoading}
        className={`${styles.toggle} ${isUrdu ? styles.active : ''}`}
        aria-label={isUrdu ? 'Switch to English' : 'Switch to Urdu'}
        aria-pressed={isUrdu}
      >
        {isLoading ? (
          <span className={styles.spinner}></span>
        ) : (
          <span className={styles.icon}>
            {isUrdu ? 'EN' : 'اردو'}
          </span>
        )}
        <span className={styles.label}>
          {isLoading ? 'Translating...' : isUrdu ? 'English' : 'Urdu'}
        </span>
      </button>

      {isUrdu && (
        <span className={styles.rtlIndicator}>
          RTL Mode
        </span>
      )}
    </div>
  );
}
