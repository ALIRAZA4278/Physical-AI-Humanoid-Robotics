import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

// Hero Section Component
function HeroSection() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGrid}></div>
      </div>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>
            <span>🤖</span> Physical AI & Robotics Course
          </div>
          <h1 className={styles.heroTitle}>
            Master <span className={styles.gradient}>Humanoid Robotics</span>
            <br />with ROS 2 & AI
          </h1>
          <p className={styles.heroSubtitle}>
            A comprehensive 4-module journey from ROS 2 fundamentals to building
            voice-controlled autonomous humanoid robots using NVIDIA Isaac and cutting-edge AI.
          </p>
          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/docs/intro">
              Start Learning →
            </Link>
            <Link
              className={clsx('button button--lg', styles.secondaryButton)}
              to="/docs/appendix/hardware-requirements">
              View Requirements
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>14</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>50+</span>
              <span className={styles.statLabel}>Code Examples</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>ROS 2</span>
              <span className={styles.statLabel}>Jazzy LTS</span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// Module Card Component
type ModuleProps = {
  number: string;
  title: string;
  description: string;
  topics: string[];
  icon: string;
  color: string;
  hardware?: string;
  link: string;
};

function ModuleCard({ number, title, description, topics, icon, color, hardware, link }: ModuleProps) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleHeader} style={{ background: color }}>
        <span className={styles.moduleIcon}>{icon}</span>
        <span className={styles.moduleNumber}>Module {number}</span>
      </div>
      <div className={styles.moduleBody}>
        <h3 className={styles.moduleTitle}>{title}</h3>
        <p className={styles.moduleDescription}>{description}</p>
        <ul className={styles.moduleTopics}>
          {topics.map((topic, idx) => (
            <li key={idx}>{topic}</li>
          ))}
        </ul>
        {hardware && (
          <div className={styles.hardwareBadge}>
            <span>⚡</span> {hardware}
          </div>
        )}
        <Link to={link} className={styles.moduleLink}>
          Explore Module →
        </Link>
      </div>
    </div>
  );
}

// Modules Section
function ModulesSection() {
  const modules: ModuleProps[] = [
    {
      number: '1',
      title: 'ROS 2 Fundamentals',
      description: 'Master the foundation of modern robotics with ROS 2 Jazzy.',
      topics: ['Nodes & Topics', 'Services & Actions', 'URDF Descriptions', 'Python rclpy'],
      icon: '🔧',
      color: 'linear-gradient(135deg, #0891b2, #06b6d4)',
      link: '/docs/module-1-ros2/architecture',
    },
    {
      number: '2',
      title: 'Digital Twin Simulation',
      description: 'Build realistic humanoid simulations with Gazebo and Unity.',
      topics: ['Gazebo Harmonic', 'SDF Modeling', 'Sensor Simulation', 'Unity Integration'],
      icon: '🌐',
      color: 'linear-gradient(135deg, #f97316, #fb923c)',
      link: '/docs/module-2-simulation/gazebo-setup',
    },
    {
      number: '3',
      title: 'NVIDIA Isaac AI',
      description: 'Deploy GPU-accelerated perception and navigation.',
      topics: ['Isaac Sim 5.0', 'cuVSLAM', 'nvblox Mapping', 'Nav2 Navigation'],
      icon: '🧠',
      color: 'linear-gradient(135deg, #84cc16, #a3e635)',
      hardware: 'RTX 3070+ Required',
      link: '/docs/module-3-isaac/isaac-setup',
    },
    {
      number: '4',
      title: 'VLA Capstone',
      description: 'Build a complete voice-controlled autonomous humanoid.',
      topics: ['Whisper ASR', 'LLM Planning', 'Vision-Language-Action', 'Full Pipeline'],
      icon: '🎤',
      color: 'linear-gradient(135deg, #7c3aed, #a78bfa)',
      hardware: 'RTX 3070+ Required',
      link: '/docs/module-4-vla/whisper',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Learning Path</h2>
          <p className={styles.sectionSubtitle}>
            Four progressive modules taking you from beginner to building autonomous humanoid robots
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <ModuleCard key={idx} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

// Technology Stack Section
function TechStackSection() {
  const technologies = [
    { name: 'ROS 2 Jazzy', desc: '5-Year LTS', icon: '🤖' },
    { name: 'Python 3.11', desc: 'Isaac Compatible', icon: '🐍' },
    { name: 'Gazebo Harmonic', desc: 'Physics Sim', icon: '🌍' },
    { name: 'Isaac Sim 5.0', desc: 'Photorealistic', icon: '🎮' },
    { name: 'Nav2', desc: 'Navigation Stack', icon: '🧭' },
    { name: 'OpenAI Whisper', desc: 'Voice Recognition', icon: '🎙️' },
  ];

  return (
    <section className={styles.techSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Technology Stack</h2>
          <p className={styles.sectionSubtitle}>
            Industry-standard tools and frameworks used throughout the course
          </p>
        </div>
        <div className={styles.techGrid}>
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techCard}>
              <span className={styles.techIcon}>{tech.icon}</span>
              <h4 className={styles.techName}>{tech.name}</h4>
              <span className={styles.techDesc}>{tech.desc}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Features Section
function FeaturesSection() {
  const features = [
    {
      icon: '📚',
      title: 'Research-Backed Content',
      description: 'Every concept verified against official documentation. No outdated information.',
    },
    {
      icon: '💻',
      title: 'Hands-On Code Examples',
      description: 'Working Python code for every concept. Download and run immediately.',
    },
    {
      icon: '🎯',
      title: 'Progressive Learning',
      description: 'Each module builds on the previous. Clear prerequisites and learning paths.',
    },
    {
      icon: '🔧',
      title: 'Real Hardware Ready',
      description: 'Code tested on real ROS 2 systems. Deploy from simulation to hardware.',
    },
    {
      icon: '🌙',
      title: 'Dark Mode Support',
      description: 'Easy on the eyes during late-night coding sessions.',
    },
    {
      icon: '📱',
      title: 'Mobile Friendly',
      description: 'Read on any device. Responsive design for tablets and phones.',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Why This Course?</h2>
          <p className={styles.sectionSubtitle}>
            Built for developers who want to master humanoid robotics with modern AI
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <span className={styles.featureIcon}>{feature.icon}</span>
              <h4 className={styles.featureTitle}>{feature.title}</h4>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Prerequisites Section
function PrerequisitesSection() {
  return (
    <section className={styles.prereqSection}>
      <div className="container">
        <div className={styles.prereqCard}>
          <div className={styles.prereqContent}>
            <h2>Ready to Start?</h2>
            <p>
              Before diving in, make sure your system meets the requirements for the modules you want to complete.
            </p>
            <div className={styles.prereqGrid}>
              <div className={styles.prereqItem}>
                <h4>Modules 1-2</h4>
                <ul>
                  <li>Ubuntu 24.04 LTS</li>
                  <li>16GB RAM</li>
                  <li>Any modern CPU</li>
                  <li>No GPU required</li>
                </ul>
              </div>
              <div className={styles.prereqItem}>
                <h4>Modules 3-4</h4>
                <ul>
                  <li>Ubuntu 24.04 LTS</li>
                  <li>32GB RAM</li>
                  <li>NVIDIA RTX 3070+</li>
                  <li>8GB+ VRAM</li>
                </ul>
              </div>
            </div>
          </div>
          <div className={styles.prereqCTA}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/docs/intro">
              Begin Your Journey →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Main Home Component
export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn Physical AI and Humanoid Robotics from ROS 2 fundamentals to voice-controlled autonomous humanoids">
      <HeroSection />
      <main>
        <ModulesSection />
        <TechStackSection />
        <FeaturesSection />
        <PrerequisitesSection />
      </main>
    </Layout>
  );
}
