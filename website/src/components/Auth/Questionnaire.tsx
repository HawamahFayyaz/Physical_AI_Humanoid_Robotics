import React, { useState } from 'react';
import styles from './styles.module.css';

interface QuestionnaireProps {
  onComplete?: () => void;
  apiUrl?: string;
}

const EXPERIENCE_LEVELS = [
  { value: 'beginner', label: 'Beginner', description: 'New to robotics and AI' },
  { value: 'intermediate', label: 'Intermediate', description: 'Some experience with ROS or ML' },
  { value: 'advanced', label: 'Advanced', description: 'Professional experience in robotics' },
];

const INTEREST_OPTIONS = [
  'ROS2 Development',
  'Computer Vision',
  'Motion Planning',
  'Simulation (Gazebo/Isaac)',
  'Machine Learning',
  'Sensor Integration',
  'Humanoid Robotics',
  'Navigation & SLAM',
  'Manipulation',
  'Voice Control',
];

export default function Questionnaire({
  onComplete,
  apiUrl = '/api/auth'
}: QuestionnaireProps): JSX.Element {
  const [experienceLevel, setExperienceLevel] = useState('');
  const [interests, setInterests] = useState<string[]>([]);
  const [goals, setGoals] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleInterest = (interest: string) => {
    setInterests((prev) =>
      prev.includes(interest)
        ? prev.filter((i) => i !== interest)
        : prev.length < 5
        ? [...prev, interest]
        : prev
    );
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (!experienceLevel) {
      setError('Please select your experience level');
      return;
    }

    if (interests.length === 0) {
      setError('Please select at least one interest');
      return;
    }

    setIsLoading(true);

    try {
      const token = localStorage.getItem('auth_token');
      const response = await fetch(`${apiUrl}/questionnaire`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          experience_level: experienceLevel,
          interests,
          goals: goals || null,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to save preferences');
      }

      onComplete?.();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save preferences');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <h2 className={styles.title}>Personalize Your Experience</h2>
      <p className={styles.subtitle}>Help us tailor content to your needs</p>

      <form onSubmit={handleSubmit} className={styles.form}>
        {error && <div className={styles.error}>{error}</div>}

        <div className={styles.inputGroup}>
          <label>Experience Level</label>
          <div className={styles.radioGroup}>
            {EXPERIENCE_LEVELS.map((level) => (
              <label
                key={level.value}
                className={`${styles.radioOption} ${
                  experienceLevel === level.value ? styles.selected : ''
                }`}
              >
                <input
                  type="radio"
                  name="experience"
                  value={level.value}
                  checked={experienceLevel === level.value}
                  onChange={(e) => setExperienceLevel(e.target.value)}
                />
                <span className={styles.radioLabel}>{level.label}</span>
                <span className={styles.radioDescription}>{level.description}</span>
              </label>
            ))}
          </div>
        </div>

        <div className={styles.inputGroup}>
          <label>Interests (select up to 5)</label>
          <div className={styles.checkboxGroup}>
            {INTEREST_OPTIONS.map((interest) => (
              <label
                key={interest}
                className={`${styles.checkboxOption} ${
                  interests.includes(interest) ? styles.selected : ''
                }`}
              >
                <input
                  type="checkbox"
                  checked={interests.includes(interest)}
                  onChange={() => toggleInterest(interest)}
                />
                {interest}
              </label>
            ))}
          </div>
        </div>

        <div className={styles.inputGroup}>
          <label htmlFor="goals">Learning Goals (optional)</label>
          <textarea
            id="goals"
            value={goals}
            onChange={(e) => setGoals(e.target.value)}
            placeholder="What do you hope to achieve? e.g., Build a humanoid robot, Learn ROS2..."
            rows={3}
          />
        </div>

        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Saving...' : 'Complete Setup'}
        </button>
      </form>
    </div>
  );
}
