/**
 * LoadingIndicator component - Typing animation for loading states.
 */

import React from 'react';
import styles from './styles.module.css';

interface LoadingIndicatorProps {
  /** Optional size variant */
  size?: 'small' | 'medium' | 'large';
  /** Optional label for accessibility */
  label?: string;
}

/**
 * Animated loading indicator with typing dots.
 */
export function LoadingIndicator({
  size = 'medium',
  label = 'Loading...',
}: LoadingIndicatorProps): JSX.Element {
  return (
    <div
      className={`${styles.loadingIndicator} ${styles[`loading${size.charAt(0).toUpperCase() + size.slice(1)}`]}`}
      role="status"
      aria-label={label}
    >
      <div className={styles.loadingDots}>
        <span></span>
        <span></span>
        <span></span>
      </div>
      <span className={styles.srOnly}>{label}</span>
    </div>
  );
}

export default LoadingIndicator;
