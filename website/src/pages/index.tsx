import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

type ModuleItem = {
  title: string;
  emoji: string;
  description: string;
  chapters: number;
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Introduction to Physical AI',
    emoji: '🤖',
    description: 'Understand Physical AI concepts, embodied intelligence, and the hardware ecosystem powering modern humanoid robots.',
    chapters: 2,
    link: '/docs/introduction/what-is-physical-ai',
  },
  {
    title: 'ROS 2 Fundamentals',
    emoji: '🔧',
    description: 'Master ROS 2 architecture, nodes, topics, services, and the core communication patterns for robotics development.',
    chapters: 3,
    link: '/docs/ros2/ros2-architecture',
  },
  {
    title: 'Digital Twin & Simulation',
    emoji: '🌐',
    description: 'Build and simulate robots using Gazebo Fortress, URDF models, and realistic sensor simulations.',
    chapters: 3,
    link: '/docs/digital-twin/gazebo-simulation',
  },
  {
    title: 'NVIDIA Isaac Platform',
    emoji: '🎮',
    description: 'Leverage Isaac Sim, Isaac ROS, and Nav2 for GPU-accelerated perception and navigation.',
    chapters: 3,
    link: '/docs/isaac/isaac-sim-introduction',
  },
  {
    title: 'VLA & Capstone Project',
    emoji: '🎯',
    description: 'Integrate voice commands, kinematics, and build a complete autonomous humanoid robot system.',
    chapters: 3,
    link: '/docs/vla-capstone/humanoid-kinematics',
  },
];

function Module({title, emoji, description, chapters, link}: ModuleItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.moduleCard}>
        <div className={styles.moduleEmoji}>{emoji}</div>
        <Heading as="h3" className={styles.moduleTitle}>{title}</Heading>
        <p className={styles.moduleDescription}>{description}</p>
        <div className={styles.moduleFooter}>
          <span className={styles.chapterCount}>{chapters} chapters</span>
          <Link className={styles.moduleLink} to={link}>
            Start Learning →
          </Link>
        </div>
      </div>
    </div>
  );
}

function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">
          A comprehensive guide to building intelligent humanoid robots with ROS 2,
          NVIDIA Isaac, and modern AI techniques.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction/what-is-physical-ai">
            Start Reading →
          </Link>
          <Link
            className="button button--outline button--secondary button--lg"
            to="https://github.com/your-username/Physical_AI_Humanoid_Robotics">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageModules() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">Curriculum Overview</Heading>
          <p>14 comprehensive chapters across 5 modules</p>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className={styles.feature}>
              <div className={styles.featureIcon}>📚</div>
              <Heading as="h3">Production-Ready Code</Heading>
              <p>Every chapter includes tested Python and ROS 2 code examples you can run immediately.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <div className={styles.featureIcon}>🎓</div>
              <Heading as="h3">Hands-On Learning</Heading>
              <p>Assessments, coding exercises, and capstone projects to solidify your understanding.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <div className={styles.featureIcon}>🚀</div>
              <Heading as="h3">Industry-Standard Tools</Heading>
              <p>Learn the same technologies used by Tesla, Figure, and leading robotics companies.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A comprehensive guide to building intelligent humanoid robots with ROS 2, NVIDIA Isaac, and modern AI techniques.">
      <HomepageHeader />
      <main>
        <HomepageModules />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
